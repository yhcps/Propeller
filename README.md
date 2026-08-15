# Propeller - 水下机器人控制系统

基于树莓派 + STM32F103 的 8 路推进器水下机器人控制系统。支持 IMU 姿态闭环、
网页控制、SPI 链路诊断，以及 SPI / ST-Link 双控制途径切换。

## 项目结构

```
imu_ws/
├── src/imu_serial_driver/   # ROS2 IMU 串口驱动（C）
├── web_server/              # 网页控制服务（Python）
│   ├── imu_web_server.py    # 主服务：IMU 数据 + 推进器控制 + 控制途径切换
│   ├── esc_spi.py           # SPI 电调通信（全双工，MISO 状态回读）
│   ├── stlink_control.py    # ST-Link 直写 PWM（OpenOCD 封装）
│   ├── imu_controller.py    # IMU PID 闭环
│   ├── thruster_mixer.py    # 8 路混控
│   └── static/index.html    # 网页界面（含控制途径切换）
├── pc_host/                 # 旧版桌面上位机（可选）
├── stlink_pwm.sh            # ST-Link 命令行 PWM 调试脚本
├── gitd.py                  # git daemon OTA 脚本（Pi 侧）
└── ota_push.py              # SFTP 增量推送 + 重启（Pi 侧，备用）
```
## WEB前端网址

电脑有线网卡地址网址：192.168.50.2

192.168.50.1:8080


## 控制途径切换（SPI / ST-Link）

网页左侧「控制途径」面板可自由切换两种推进器控制方式：

| 途径 | 说明 | 适用场景 |
|------|------|----------|
| **SPI 通道** | 经 `spidev` 下发 26 字节控制帧到 STM32 SPI 从机，40Hz 闭环 | 常规闭环控制（默认） |
| **ST-Link 调试** | 经 ST-Link（OpenOCD）直接写 TIM1/TIM4 的 CCR 寄存器 | 硬件验证、排查个别通道 |

- 切换接口：`POST /api/transport {"mode": "spi" | "stlink"}`
- 切换前会自动把 8 路油门收回中位，避免切换瞬间电机悬空。
- ST-Link 途径有最小写间隔节流（0.5s），仅适合低频调试，不适合 40Hz 闭环。
- 面板实时显示当前途径与 ST-Link 连接状态（设备是否在线、最近一次写入结果）。

ST-Link 直写寄存器地址（与 `stlink_pwm.sh` 一致）：

| 通道 | 定时器 | GPIO | CCR 地址 |
|------|--------|------|----------|
| 0 | TIM1_CH1 | PA8 | 0x40012C34 |
| 1 | TIM1_CH2 | PA9 | 0x40012C38 |
| 2 | TIM1_CH3 | PA10 | 0x40012C3C |
| 3 | TIM1_CH4 | PA11 | 0x40012C40 |
| 4 | TIM4_CH1 | PB6 | 0x40000834 |
| 5 | TIM4_CH2 | PB7 | 0x40000838 |
| 6 | TIM4_CH3 | PB8 | 0x4000083C |
| 7 | TIM4_CH4 | PB9 | 0x40000840 |

## SPI 链路诊断（MISO 回读）

STM32 固件为全双工：每收一帧 26 字节控制帧，MISO 同时回传 26 字节状态帧。
Pi 侧 `esc_spi.py` 的 `send_frame()` 返回 `(frame, miso)`，`SPIStatus.from_miso()`
解析回传数据。网页控制面板实时显示链路状态：

- **绿色「已连接」**：MISO 回读有效，帧头 + CRC 正确
- **橙色「仅发送(无回读)」**：SPI 在发送但 MISO 无有效回读（检查 MISO 接线/固件版本）
- **灰色「模拟」**：无物理 SPI 设备

后台 `spi_status_monitor()` 线程周期检测状态变化并写 `/tmp/spi_status.log`。

回传帧内容：`spi_connected`、`spi_ok_count`（帧校验成功）、`spi_fail_count`（失败）、
`spi_miso_health`（单行健康摘要）。

## ST-Link 调试机制

### 命令行（stlink_pwm.sh）

```bash
bash stlink_pwm.sh read          # 读取全部 8 路 PWM 当前值
bash stlink_pwm.sh set <ch> <us> # 设置通道 <ch>(0-7) 为 <us>us
bash stlink_pwm.sh all <us>      # 设置全部 8 路
bash stlink_pwm.sh stop          # 全部回到中位 1500us
bash stlink_pwm.sh ramp <ch>     # 通道扫频 1000→2000→1000us
```

### Python 模块（stlink_control.py）

供 web 服务在 ST-Link 途径下调用，封装一次 OpenOCD 会话写 8 路 CCR，
含最小写间隔节流与连接状态查询（`stlink_detect()` / `get_stlink_status()`）。

## GitHub OTA 机制

树莓派通过 git daemon 从本地 bare repo 拉取代码更新（详见 `AGENTS.md`）。

```
本地 Propeller_repo ──git push──▶ propeller-bare.git ──git pull──▶ Pi /home/han/imu_ws
                                  git://192.168.50.2:9418
```

| 命令 | 作用 |
|------|------|
| `python3 gitd.py start` | 启动 git daemon |
| `python3 gitd.py push --restart` | 推送 + Pi 拉取 + 重启服务 |
| `python3 gitd.py stop` | 停止 daemon |

备用 SFTP 模式：`ota_push.py`（增量推送 + commit + 重启）。

> 注：`gitd.py` / `ota_push.py` 位于树莓派侧 `/home/han/imu_ws/`，本地开发机
> 通过 `gitd.py push --restart` 一键触发。

## STM32 固件改动记录

固件源码在 `Propeller_repo/STM32F103C8T6/`（GitHub `stm32-firmware` 分支）。近期改动：

- **PB6-PB9 AF_PP**：`esc_spi.c` 的 `ESC_PWM_Init()` 显式配置 PB6-9 为 AF 推挽，
  修复 `HAL_TIM_PWM_MspInit()` 缺失导致的 TIM4 PWM 无输出。
- **PB12-PB15 AF_PP**：`spi_slave.c` 的 `SPI_Slave_Init()` 将 SPI2 引脚从输入改为
  AF 推挽，并把 NSS 改为软件模式（`SPI_NSS_SOFT`）。
- **EMA 输出平滑**：`ESC_ApplyToPWM()` 增加 EMA 低通滤波 + 每帧最大跳变限幅
  （100us），抑制抖动与输出峰值。

## IMU 闭环控制

三种控制模式（网页左侧切换）：

| 模式 | 说明 |
|------|------|
| **manual** | 手动：键盘/摇杆直接控制 8 路推进器 |
| **imu_hold** | IMU 保持：PID 根据姿态误差自动稳姿 |
| **hybrid** | 混合：摇杆设定目标姿态，IMU PID 自动跟踪修正 |

数据流：`IMU → PID(imu_controller.py) → 8路混控 → SPI/ST-Link → STM32 → 推进器`

## 快速启动

```bash
/home/han/imu_ws/web_server/start_imu_web.sh
```

浏览器打开：`http://<树莓派IP>:8080`

## 依赖

- ROS 2 Jazzy
- Python 3
- IMU 串口：`/dev/ttyUSB0`
- SPI 电调：`spidev`（`sudo apt install python3-spidev`）
- ST-Link 调试：`openocd` + ST-Link 调试器
