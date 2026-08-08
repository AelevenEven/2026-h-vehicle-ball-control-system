# K230 钢球视觉与舵机协同

## K230 端部署

将以下内容复制到 K230 的 `/data/`：

- `main.py`
- `det_video_1_3.py`
- 整个 `mp_deployment_source` 文件夹

最终模型配置路径应为：

`/data/mp_deployment_source/deploy_config.json`

脚本也兼容 `/sdcard/mp_deployment_source/`。

## 串口接线

- K230 pin 11（UART2 TX）→ MSPM0 PB7（UART_1 RX）
- K230 pin 12（UART2 RX）← MSPM0 PB6（UART_1 TX，目前可不接）
- 两块板必须共地
- 波特率：115200，8N1

发送协议：

`BALL,序号,状态,位置mm,X误差px,Y误差px,置信度千分数,中心X,中心Y\r\n`

例如：

`BALL,25,1,-36,-138,5,927,502,365`

## 必须进行的标定

在 `det_video_1_3.py` 顶部修改：

- `ROD_NEG_END_PX`：摆杆负方向端点在 1280×720 图像中的坐标
- `ROD_POS_END_PX`：摆杆正方向端点坐标
- `ROD_LENGTH_CM`：两标定端点间的实际长度

屏幕黄线应与摆杆中心线重合。位置正负相反时，交换两个端点。

### 多点位置标定

屏幕现在同时显示：

- `RAW`：沿摆杆投影后、尚未校正的原始位置
- `POS`：五点插值校正后的实际位置，也是 UART 发给主控的位置

保持摄像头和水管相对位置固定。至少将钢球依次静止放在：

`-5 cm、-1 cm、0 cm、+1 cm、+5 cm`

记录每个位置稳定后的 `RAW`，然后修改：

```python
POSITION_RAW_CALIBRATION_CM = (-11.0, -1.0, 0.0, 1.0, 11.0)
POSITION_TRUE_CALIBRATION_CM = (-5.0, -1.0, 0.0, 1.0, 5.0)
```

第一行必须填写实际测到的 `RAW`，并保持严格递增；第二行是对应的水管真实刻度。
程序允许增加标定点。为了覆盖赛题25 cm水管的完整有效范围，建议最终使用：

```python
POSITION_RAW_CALIBRATION_CM = (
    实测负端, 实测负5, 实测负1, 实测0, 实测正1, 实测正5, 实测正端
)
POSITION_TRUE_CALIBRATION_CM = (-12.0, -5.0, -1.0, 0.0, 1.0, 5.0, 12.0)
```

当前默认允许在最外侧标定段上继续外推，并最终按钢球中心的物理可动范围限制到
`-12..+12 cm`。在采集到 ±12 cm 标定点后，整个有效范围才能可靠用于控制。

摄像头、水管或支架位置发生变化后，必须重新记录全部 `RAW` 标定值。

主控端 `Hardware/ball_control.c` 中：

- `BALL_STEPPER_DIRECTION_SIGN`：升降方向相反时改为 `-1`
- `BALL_POSITION_KP_PULSE_PER_MM`、`BALL_VELOCITY_KD_PULSE_PER_MM_S`：
  钢球外环的比例和速度阻尼参数
- `BALL_STEPPER_MAX_POSITION_PULSES`：按连杆安全角度设置的最大偏转
- `BALL_STEPPER_MAX_STEP_PULSES`：每 50 ms 允许的最大目标变化

首次测试必须架空或拆开连杆，手动将摆杆调平后再按键置零，确认 X42S 方向和
安全行程，最后才把钢球放到摆杆上。
