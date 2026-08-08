# -*- coding: utf-8 -*-
"""
K230 钢球检测、摆杆角度补偿和 MSPM0 协同控制程序。

UART frame (ASCII, 115200 8N1):
BALL,seq,status,position_mm,error_x,error_y,confidence_permille,cx,cy\r\n

状态为 1 表示正在跟踪有效钢球，状态为 0 表示当前没有有效钢球。
坐标均基于 RGB888P_SIZE，而不是 LCD 显示分辨率；位置正负以摆杆物理中心 O 为基准，右侧为正。
"""

import gc
import math
import os
import sys
import time

from machine import FPIOA, UART
from libs.PlatTasks import DetectionApp
from libs.PipeLine import PipeLine
from libs.Utils import *


# ---------- Camera, model and display ----------
DISPLAY_MODE = "lcd"
RGB888P_SIZE = [1280, 720]
MODEL_ROOT_CANDIDATES = (
    "/data/mp_deployment_source/",
    "/sdcard/mp_deployment_source/",
)
DEBUG_MODE = 0

# The trained category in deploy_config.json is "ball2".
BALL_LABELS = ("ball2", "ball", "steel_ball", "steelball")

# A detection within this confidence distance from the best detection may win
# by being nearer the image center. This avoids jumping to a remote false box.
BEST_CONFIDENCE_WINDOW = 0.05

# ---------- Alpha-beta tracking filter ----------
# Higher alpha follows position faster. Beta controls the velocity correction.
FILTER_ALPHA = 0.65
FILTER_BETA = 0.08
FILTER_RESET_AFTER_MISSES = 5

# ---------- Direct calibration from +/-5 cm black marks ----------
# Place the ball exactly on each black +/-5 cm mark on the calibration board.
# Read the cx value from the LCD OSD line "ball (cx,cy)" and update below.
CALIB_NEG5_CX = 374     # ball cx when placed at the -5 cm red mark
CALIB_POS5_CX = 901     # ball cx when placed at the +5 cm red mark
CALIB_YC = 360          # rod centreline Y in the 1280x720 camera image
ROD_LENGTH_CM = 25.0
PIPE_WIDTH_CM = 2.0

# Derived values — DO NOT EDIT below this line.
_CALIB_PX_PER_CM = (CALIB_POS5_CX - CALIB_NEG5_CX) / 10.0
_CALIB_CENTER_X = (CALIB_POS5_CX + CALIB_NEG5_CX) / 2.0
_CALIB_HALF_LEN_PX = ROD_LENGTH_CM * 0.5 * _CALIB_PX_PER_CM

ROD_NEG_END_PX = (_CALIB_CENTER_X - _CALIB_HALF_LEN_PX, float(CALIB_YC))
ROD_POS_END_PX = (_CALIB_CENTER_X + _CALIB_HALF_LEN_PX, float(CALIB_YC))

# ---------- Multi-point position calibration ----------
# With correct rod endpoints, RAW should already be close to true cm.
# Keep 1:1 linear; adjust only if lens distortion is significant.
POSITION_RAW_CALIBRATION_CM = (-12.0, -5.0, 0.0, 5.0, 12.0)
POSITION_TRUE_CALIBRATION_CM = (-12.0, -5.0, 0.0, 5.0, 12.0)

# The H-problem pipe is 25 cm long and the 1 cm ball center can move to about
# +/-12 cm. Never clamp at +/-5 cm: the controller must still see overshoot.
POSITION_EXTRAPOLATE_OUTSIDE = True
POSITION_PHYSICAL_LIMIT_CM = 12.0
# ---------- K230 UART2 ----------
# Wiring:
#   K230 pin 11 (TX) -> MSPM0 PB7 / UART_1 RX
#   K230 pin 12 (RX) <- MSPM0 PB6 / UART_1 TX (optional at present)
#   K230 GND          --- MSPM0 GND
UART_ID = UART.UART2
UART_TX_PIN = 11
UART_RX_PIN = 12
UART_BAUD = 115200
UART_SEND_PERIOD_MS = 25

# OSD ARGB colors.
COLOR_RED = (255, 255, 0, 0)
COLOR_GREEN = (255, 0, 255, 0)
COLOR_YELLOW = (255, 255, 255, 0)
COLOR_WHITE = (255, 255, 255, 255)


def find_model_root():
    for root in MODEL_ROOT_CANDIDATES:
        try:
            with open(root + "deploy_config.json", "r"):
                return root
        except Exception:
            pass
    raise RuntimeError("deploy_config.json not found under /data or /sdcard")


def init_uart():
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
    # The CanMV default data format is 8N1.
    return UART(UART_ID, UART_BAUD)


def ball_class_ids(labels):
    ids = []
    for index, label in enumerate(labels):
        if str(label).lower() in BALL_LABELS:
            ids.append(index)
    # This model has one steel-ball class. Keep class 0 usable if its online
    # label is renamed without updating BALL_LABELS.
    if not ids and len(labels) == 1:
        ids.append(0)
    return ids


def normalize_detections(result):
    """
    Convert CanMV 1.3+ DetectionApp output to the legacy row format.

    Current firmware returns:
        {"boxes": [[x1,y1,x2,y2], ...],
         "scores": [score, ...],
         "idx": [class_id, ...]}

    Older exports returned [class_id, score, x1, y1, x2, y2] rows.
    Supporting both keeps the script usable if the board firmware changes.
    """
    if result is None:
        return ()
    if not isinstance(result, dict):
        return result

    boxes = result.get("boxes", ())
    scores = result.get("scores", ())
    class_ids = result.get("idx", ())
    count = min(len(boxes), len(scores), len(class_ids))
    rows = []
    for index in range(count):
        box = boxes[index]
        if len(box) < 4:
            continue
        rows.append((
            class_ids[index],
            scores[index],
            box[0],
            box[1],
            box[2],
            box[3],
        ))
    return rows


def select_one_ball(detections, valid_class_ids, frame_center):
    """Return one [class, score, x1, y1, x2, y2] detection or None."""
    candidates = []
    for det in detections:
        if len(det) < 6:
            continue
        class_id = int(det[0])
        if class_id not in valid_class_ids:
            continue
        score = float(det[1])
        x1, y1, x2, y2 = map(float, det[2:6])
        if x2 <= x1 or y2 <= y1:
            continue
        cx = (x1 + x2) * 0.5
        cy = (y1 + y2) * 0.5
        dx = cx - frame_center[0]
        dy = cy - frame_center[1]
        candidates.append((det, score, dx * dx + dy * dy))

    if not candidates:
        return None

    best_score = max(item[1] for item in candidates)
    near_best = [
        item for item in candidates
        if item[1] >= best_score - BEST_CONFIDENCE_WINDOW
    ]
    # Center distance is primary only among nearly equal-confidence boxes.
    near_best.sort(key=lambda item: (item[2], -item[1]))
    return near_best[0][0]


class BallFilter:
    """Two-dimensional alpha-beta filter with measured frame interval."""

    def __init__(self, alpha, beta):
        self.alpha = alpha
        self.beta = beta
        self.valid = False
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = 0
        self.misses = 0

    def update(self, x, y, now_ms):
        self.misses = 0
        if not self.valid:
            self.x = x
            self.y = y
            self.vx = 0.0
            self.vy = 0.0
            self.last_ms = now_ms
            self.valid = True
        else:
            dt_ms = ticks_diff(now_ms, self.last_ms)
            self.last_ms = now_ms
            if dt_ms <= 0 or dt_ms > 250:
                self.x = x
                self.y = y
                self.vx = 0.0
                self.vy = 0.0
            else:
                dt = dt_ms * 0.001
                predicted_x = self.x + self.vx * dt
                predicted_y = self.y + self.vy * dt
                residual_x = x - predicted_x
                residual_y = y - predicted_y
                self.x = predicted_x + self.alpha * residual_x
                self.y = predicted_y + self.alpha * residual_y
                self.vx += self.beta * residual_x / dt
                self.vy += self.beta * residual_y / dt
        return self.x, self.y, self.vx, self.vy

    def miss(self):
        self.misses += 1
        if self.misses >= FILTER_RESET_AFTER_MISSES:
            self.valid = False
            self.vx = 0.0
            self.vy = 0.0


def project_to_rod_cm(x, y):
    """Return the uncorrected RAW coordinate along the rod."""
    dx = ROD_POS_END_PX[0] - ROD_NEG_END_PX[0]
    dy = ROD_POS_END_PX[1] - ROD_NEG_END_PX[1]
    rod_pixels = math.sqrt(dx * dx + dy * dy)
    if rod_pixels < 1.0 or ROD_LENGTH_CM <= 0.0:
        raise ValueError("invalid rod calibration")

    ux = dx / rod_pixels
    uy = dy / rod_pixels
    center_x = (ROD_NEG_END_PX[0] + ROD_POS_END_PX[0]) * 0.5
    center_y = (ROD_NEG_END_PX[1] + ROD_POS_END_PX[1]) * 0.5
    projected_pixels = (x - center_x) * ux + (y - center_y) * uy
    # RAW is intentionally not clamped. The initial rod endpoints may be only
    # approximate, and clipping here would make end-region calibration
    # impossible. Only the final corrected physical position is limited.
    return projected_pixels * ROD_LENGTH_CM / rod_pixels


def project_velocity_to_rod_cm_s(vx, vy):
    dx = ROD_POS_END_PX[0] - ROD_NEG_END_PX[0]
    dy = ROD_POS_END_PX[1] - ROD_NEG_END_PX[1]
    rod_pixels = math.sqrt(dx * dx + dy * dy)
    if rod_pixels < 1.0 or ROD_LENGTH_CM <= 0.0:
        return 0.0
    ux = dx / rod_pixels
    uy = dy / rod_pixels
    projected_pixels_s = vx * ux + vy * uy
    return projected_pixels_s * ROD_LENGTH_CM / rod_pixels


def validate_position_calibration():
    raw_points = POSITION_RAW_CALIBRATION_CM
    true_points = POSITION_TRUE_CALIBRATION_CM

    if len(raw_points) != len(true_points) or len(raw_points) < 2:
        raise ValueError("position calibration point counts do not match")
    for index in range(len(raw_points) - 1):
        if raw_points[index + 1] <= raw_points[index]:
            raise ValueError("RAW calibration values must increase")
        if true_points[index + 1] <= true_points[index]:
            raise ValueError("TRUE calibration values must increase")


def calibration_segment(raw_position_cm):
    raw_points = POSITION_RAW_CALIBRATION_CM
    last_index = len(raw_points) - 1

    if raw_position_cm <= raw_points[0]:
        if not POSITION_EXTRAPOLATE_OUTSIDE:
            return -1
        return 0
    if raw_position_cm >= raw_points[last_index]:
        if not POSITION_EXTRAPOLATE_OUTSIDE:
            return last_index
        return last_index - 1

    segment = 0
    while raw_position_cm > raw_points[segment + 1]:
        segment += 1
    return segment


def calibrate_position_and_velocity_cm(raw_position_cm, raw_velocity_cm_s):
    """Piecewise-calibrate position and apply the same local slope to speed."""
    raw_points = POSITION_RAW_CALIBRATION_CM
    true_points = POSITION_TRUE_CALIBRATION_CM
    last_index = len(raw_points) - 1
    segment = calibration_segment(raw_position_cm)

    if segment < 0:
        return true_points[0], 0.0
    if segment >= last_index:
        return true_points[last_index], 0.0

    raw_span = raw_points[segment + 1] - raw_points[segment]
    true_span = true_points[segment + 1] - true_points[segment]
    ratio = (raw_position_cm - raw_points[segment]) / raw_span
    position_cm = true_points[segment] + ratio * true_span
    velocity_cm_s = raw_velocity_cm_s * true_span / raw_span

    position_cm = max(
        -POSITION_PHYSICAL_LIMIT_CM,
        min(POSITION_PHYSICAL_LIMIT_CM, position_cm),
    )
    if ((position_cm <= -POSITION_PHYSICAL_LIMIT_CM and velocity_cm_s < 0.0)
            or (position_cm >= POSITION_PHYSICAL_LIMIT_CM
                and velocity_cm_s > 0.0)):
        velocity_cm_s = 0.0
    return position_cm, velocity_cm_s


def scale_point(x, y, display_size):
    return (
        int(x * display_size[0] / RGB888P_SIZE[0]),
        int(y * display_size[1] / RGB888P_SIZE[1]),
    )


def draw_cross(osd_img, x, y, size=10, color=COLOR_GREEN):
    osd_img.draw_line(x - size, y, x + size, y, color=color, thickness=3)
    osd_img.draw_line(x, y - size, x, y + size, color=color, thickness=3)


def draw_overlay(osd_img, display_size, detection, filtered_xy,
                 raw_position_cm, position_cm, velocity_cm_s):
    osd_img.clear()

    center_y = float(CALIB_YC)

    # ---- origin (x=0) yellow cross ----
    image_center = scale_point(
        _CALIB_CENTER_X, center_y, display_size
    )
    draw_cross(osd_img, image_center[0], image_center[1], 10, color=COLOR_YELLOW)

    if detection is None:
        osd_img.draw_string_advanced(
            8, 8, 24, "BALL LOST", color=COLOR_RED
        )
        return

    x1, y1, x2, y2 = map(float, detection[2:6])
    box_a = scale_point(x1, y1, display_size)
    box_b = scale_point(x2, y2, display_size)
    osd_img.draw_rectangle(
        box_a[0], box_a[1],
        max(1, box_b[0] - box_a[0]),
        max(1, box_b[1] - box_a[1]),
        color=COLOR_RED, thickness=3
    )

    cx, cy = filtered_xy
    display_center = scale_point(cx, cy, display_size)
    draw_cross(osd_img, display_center[0], display_center[1], 12)

    error_x = cx - _CALIB_CENTER_X
    error_y = cy - center_y
    score = float(detection[1])
    osd_img.draw_string_advanced(
        8, 8, 22,
        "ball ({},{}) conf:{:.2f}".format(int(cx), int(cy), score),
        color=COLOR_GREEN
    )
    osd_img.draw_string_advanced(
        8, 36, 22,
        "RAW:{:+.2f}cm POS:{:+.2f}cm".format(
            raw_position_cm, position_cm
        ),
        color=COLOR_WHITE
    )
    osd_img.draw_string_advanced(
        8, 64, 22,
        "V:{:+.1f}cm/s err:{:+d},{:+d}".format(
            velocity_cm_s, int(error_x), int(error_y)
        ),
        color=COLOR_WHITE
    )


def ticks_diff(now, previous):
    try:
        return time.ticks_diff(now, previous)
    except AttributeError:
        return now - previous


def send_ball_frame(uart, sequence, position_cm, error_x, error_y,
                    confidence, center_x, center_y):
    frame = "BALL,{},{:.2f},{},{},{:.2f},{},{}\r\n".format(
        sequence,
        position_cm,
        int(error_x),
        int(error_y),
        confidence,
        int(center_x),
        int(center_y),
    )
    uart.write(frame)


def main():
    pipeline = None
    detection_app = None
    uart = None

    try:
        print("1")
        validate_position_calibration()
        print("2")
        model_root = find_model_root()
        print("3", model_root)
        deploy_conf = read_json(model_root + "deploy_config.json")
        kmodel_path = model_root + deploy_conf["kmodel_path"]
        labels = deploy_conf["categories"]
        model_type = deploy_conf["model_type"]
        anchors = []
        if model_type == "AnchorBaseDet":
            anchors = (
                deploy_conf["anchors"][0]
                + deploy_conf["anchors"][1]
                + deploy_conf["anchors"][2]
            )

        # 1. Camera and display.
        print("4")
        pipeline = PipeLine(
            rgb888p_size=RGB888P_SIZE, display_mode=DISPLAY_MODE
        )
        pipeline.create()
        print("5")
        display_size = pipeline.get_display_size()
        print("6", display_size)

        # 2. Kmodel and its preprocessing.
        detection_app = DetectionApp(
            "video",
            kmodel_path,
            labels,
            deploy_conf["img_size"],
            anchors,
            model_type,
            deploy_conf["confidence_threshold"],
            deploy_conf["nms_threshold"],
            RGB888P_SIZE,
            display_size,
            debug_mode=DEBUG_MODE,
        )
        detection_app.config_preprocess()

        # 3. UART is initialized after model preprocessing as required.
        uart = init_uart()

        valid_class_ids = ball_class_ids(labels)
        frame_center = (_CALIB_CENTER_X, float(CALIB_YC))
        ball_filter = BallFilter(FILTER_ALPHA, FILTER_BETA)
        sequence = 0
        last_send_ms = time.ticks_ms()

        while True:
            try:
                os.exitpoint()
            except AttributeError:
                pass

            image = pipeline.get_frame()
            detections = normalize_detections(detection_app.run(image))
            now_ms = time.ticks_ms()
            selected = select_one_ball(
                detections,
                valid_class_ids,
                frame_center,
            )

            if selected is not None:
                raw_x = (float(selected[2]) + float(selected[4])) * 0.5
                raw_y = (float(selected[3]) + float(selected[5])) * 0.5
                filtered_x, filtered_y, filtered_vx, filtered_vy = (
                    ball_filter.update(raw_x, raw_y, now_ms)
                )
                error_x = filtered_x - frame_center[0]
                error_y = filtered_y - frame_center[1]
                raw_position_cm = project_to_rod_cm(filtered_x, filtered_y)
                raw_velocity_cm_s = project_velocity_to_rod_cm_s(
                    filtered_vx, filtered_vy
                )
                position_cm, velocity_cm_s = (
                    calibrate_position_and_velocity_cm(
                        raw_position_cm, raw_velocity_cm_s
                    )
                )
                confidence = float(selected[1])
            else:
                ball_filter.miss()
                filtered_x = -1.0
                filtered_y = -1.0
                error_x = 0.0
                error_y = 0.0
                raw_position_cm = 0.0
                position_cm = 0.0
                velocity_cm_s = 0.0
                confidence = 0.0

            draw_overlay(
                pipeline.osd_img,
                display_size,
                selected,
                (filtered_x, filtered_y),
                raw_position_cm,
                position_cm,
                velocity_cm_s,
            )

            if ticks_diff(now_ms, last_send_ms) >= UART_SEND_PERIOD_MS:
                last_send_ms = now_ms
                send_ball_frame(
                    uart,
                    sequence,
                    position_cm,
                    error_x,
                    error_y,
                    confidence,
                    filtered_x,
                    filtered_y,
                )
                sequence = (sequence + 1) & 0xFFFF

            pipeline.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("steel-ball vision stopped")
    except Exception as error:
        sys.print_exception(error)
    finally:
        if detection_app is not None:
            detection_app.deinit()
        if pipeline is not None:
            pipeline.destroy()
        if uart is not None:
            uart.deinit()
        gc.collect()


if __name__ == "__main__":
    main()
