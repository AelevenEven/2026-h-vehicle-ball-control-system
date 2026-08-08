# X42S 电机轴角度与视觉透视标定

## 角度定义

本项目的“角度”统一定义为 X42S 电机轴相对软件零点转过的角度，不再换算为
摆杆的物理倾角。只要连杆安装与软件零点不变，同一电机轴角度就对应同一套
摄像机透视参数。

当前 `empty.c` 默认使用 `APP_MODE_ROD_ANGLE_CAL`。底盘保持停止，按键只控制
X42S 在安全的绝对位置之间移动。这个标定模式只按目标脉冲计算电机轴角度，
不要求 X42S 连续回传实时位置，也不要求 K230 上电。标定期间不发送 `0x11`
自动回传配置，也不发送 `0x36` 位置查询，只发送使能、清零和绝对位置命令，
避免不同 Emm 固件版本之间的查询协议差异。清零和使能命令之间保留 20 ms
间隔，防止手册中说明的串口命令粘包导致 `0xEE` 格式错误。

## 接线

- MSPM0 PB17 / UART2 TX -> X42S R/A/H
- MSPM0 PB16 / UART2 RX <- X42S T/B/L
- MSPM0 PB6 / UART1 TX -> K230 Pin12 / UART2 RX
- K230 Pin11 / UART2 TX -> MSPM0 PB7 / UART1 RX
- MSPM0、X42S、K230 必须共地

烧录串口参数为 115200、8N1。主控输出：

```text
CAL,index,target_pulses,commanded_cdeg,settled
```

- `target_pulses`：发送给 X42S 的绝对目标脉冲。
- `commanded_cdeg`：由目标脉冲换算的电机轴角度，单位 0.01 度，例如
  `-1350` 表示 `-13.50°`。
- `settled`：移动后经过 800 ms 为 1，只有此时才采集像素。

稳定后，主控还会给 K230 发送：

```text
ANGLE,sequence,angle_cdeg
```

X42S应答状态`002`表示命令接收正确，不是故障。标定模式不会因为没有实时位置
回传而显示`EMM RX TIMEOUT`；只有收到 X42S 明确返回的参数、格式或限位错误时
才停止动作。正式第三题使用目标脉冲换算角度，不依赖 `0x36` 位置回传；仍保留
X42S 命令错误、串口溢出、K230 丢球和视觉数据超时保护。

当前实车没有安装蜂鸣器，因此`Hardware/Beep.h`中的`BEEP_MODULE_ENABLE`默认
为`0`。所有蜂鸣调用均为立即返回的空操作，不会再阻塞串口接收；以后安装
PA22蜂鸣器后再将该宏改为`1`。

## 烧录后怎么采集

1. 把摆杆手动调到你定义的基准水平位置。
2. 烧录新固件，上电后 OLED 显示 `LEVEL THE ROD / FIRST KEY=ZERO`。
3. 第一次按键只做软件清零和电机使能，不移动；此阶段 K230 可以不上电。
4. 后续每按一次按键进入下一个角度。先确认电机能依次转到各角度；需要采集
   视觉数据时再打开 K230，并等待 CSV 的 `settled=1` 后记录像素。
5. 在当前角度分别把钢球放到管道刻度 `-5 cm、0 cm、+5 cm`，记录K230检测框
   中心坐标 `x,y`。
6. 再按键进入下一角度，重复三点采集。
7. 将各角度数据填入 K230 `代码.py` 的 `ROD_ANGLE_CALIBRATION`，角度字段使用
   OLED/CSV 显示的电机轴指令角度。

建议记录格式：

```text
motor_angle_deg,neg5_x,neg5_y,zero_x,zero_y,pos5_x,pos5_y
-18.00,...,...,...,...,...,...
-13.50,...,...,...,...,...,...
-9.00,...,...,...,...,...,...
-4.50,...,...,...,...,...,...
0.00,...,...,...,...,...,...
+4.50,...,...,...,...,...,...
+9.00,...,...,...,...,...,...
+13.50,...,...,...,...,...,...
+18.00,...,...,...,...,...,...
```

正方向和负方向返回相同角度时各测一次；若像素差异明显，说明机构回差较大，
需要先消除机械间隙，或以后固定从同一方向逼近采集角度。

## 怎么自己调角度范围

只修改 [Hardware/ball_control.c](Hardware/ball_control.c) 顶部的数组：

```c
static const int16_t s_rod_calibration_pulses[] = {
    0, -40, -80, -120, -160, -120, -80, -40,
    0, 40, 80, 120, 160, 120, 80, 40, 0
};
```

在当前 3200 脉冲/转设置下，理论上：

- 40 脉冲约等于电机轴 4.5°
- 80 脉冲约等于 9.0°
- 120 脉冲约等于 13.5°
- 160 脉冲约等于 18.0°

当前标定角度按 `target_pulses × 360° / 3200` 计算；因此必须保证 X42S 的
每圈脉冲设置确实为 3200。若驱动器中修改了细分数，同时修改
`ROD_CAL_PULSES_PER_REVOLUTION`。

- 机构接近限位：把最大值从 `160` 改为 `120` 或更小。
- 需要更密的点：改为每 20 脉冲一点。
- 未检查机构安全前不要超过正负 160 脉冲。
- 正负方向与实际升降相反不影响标定，不要为了改符号交换串口线。

速度与加速度也在同一位置：

```c
#define ROD_CAL_SPEED_RPM       (80U)
#define ROD_CAL_ACCELERATION    (40U)
```

## K230采集设置

`代码.py` 在采集阶段应为：

```python
ANGLE_COMPENSATION_ENABLE = True
REQUIRE_VALID_ANGLE_FOR_BALL = False
ANGLE_MIN_DEG = -20.0
ANGLE_MAX_DEG = 20.0
```

采集时保持 `REQUIRE_VALID_ANGLE_FOR_BALL=False`，避免电机移动时的短暂角度超时
禁止钢球框输出。填完多角度像素表并进入正式闭环后，再改成 `True`。

## 编译与烧录

1. 用 Keil 打开 `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`。
2. 确认 `empty.c` 的 `APP_RUN_MODE` 是 `APP_MODE_ROD_ANGLE_CAL`。
3. 点击 Rebuild，再点击 Download。
4. 也可以直接烧录 `keil/Objects/empty_LP_MSPM0G3507_nortos_keil.hex`。
5. 采集完成后，把 `APP_RUN_MODE` 改回 `APP_MODE_BALL_CONTROL`，再重新编译烧录。
