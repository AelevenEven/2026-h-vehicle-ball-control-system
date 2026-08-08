# Keil 工程说明

主工程文件为 `empty_LP_MSPM0G3507_nortos_keil.uvprojx`，目标芯片为
MSPM0G3507。打开工程后执行 Rebuild，生成文件默认位于 `Objects/`；这些文件
属于本地构建产物，不提交到 GitHub。

编译前请确认：

1. Keil 已安装 Arm Compiler 和 MSPM0 SDK/Device Pack。
2. `../ti_msp_dl_config.c` 与 `../ti_msp_dl_config.h` 来自当前
   `empty.syscfg` 的配置结果。
3. 车辆上电时保持静止，便于 MPU6050 完成零偏初始化。
4. 首次测试先架空车辆，确认左右轮方向、编码器计数方向和 X42S 行程。

命令行构建示例：

```powershell
& 'D:\Keil5\UV4\UV4.exe' -r `
  'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -j0
```

程序入口和模式选择位于仓库根目录的 `empty.c`。参数调整优先集中在
`Hardware/task2_config.h` 和 `Hardware/ball_control.c`，不要直接修改生成的
目标文件。
