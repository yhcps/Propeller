#!/usr/bin/env python3
"""IMU 反馈闭环 + 网页推进器控制服务"""

from __future__ import annotations

import json
import math
import mimetypes
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from imu_controller import ImuThrusterController, wrap_angle_deg
from thruster_mixer import MOTOR_SIGN, mix_thrusters

# SPI 参数与总线锁的唯一来源见 esc_spi；固件约束见 docs/STM32固件SPI约束.md
from esc_spi import FRAME_GAP_SEC as SPI_FRAME_GAP_SEC
from esc_spi import MOTOR_TRIM_US, SPI_SPEED_HZ, get_stm32_status, percent_to_us, spi_bus_lock

try:
    from esc_spi import ESC_SPI
except ImportError:
    ESC_SPI = None
    print("[WARN] spidev 未安装，SPI 推进器仅模拟输出（请: sudo apt install python3-spidev）")

# STLink 控制途径：绕过 SPI 从机，直接用 ST-Link(OpenOCD) 写 TIM 比较寄存器。
# 主要用于硬件验证 / 排查个别通道，低频调试用途。
import stlink_control

STATIC_DIR = Path(__file__).resolve().parent / "static"
CAM_SCRIPT = Path("/home/han/camera_stream.py")
CAM_LOG = Path("/tmp/cam.log")
CAM_PORT = 8081
_camera_proc: subprocess.Popen | None = None
_camera_lock = threading.Lock()


def _camera_running() -> bool:
    with _camera_lock:
        return _camera_proc is not None and _camera_proc.poll() is None


def _camera_status() -> dict:
    return {
        "ok": True,
        "running": _camera_running(),
        "port": CAM_PORT,
        "script_exists": CAM_SCRIPT.is_file(),
        "device_exists": Path("/dev/video0").exists(),
    }


def start_camera() -> dict:
    global _camera_proc
    with _camera_lock:
        if _camera_proc is not None and _camera_proc.poll() is None:
            return {"ok": True, "running": True, "message": "摄像头已在运行"}
        if not CAM_SCRIPT.is_file():
            return {"ok": False, "running": False, "message": f"缺少脚本: {CAM_SCRIPT}"}
        if not Path("/dev/video0").exists():
            return {"ok": False, "running": False, "message": "未找到 /dev/video0"}
        subprocess.run(["pkill", "-f", "camera_stream.py"], capture_output=True)
        time.sleep(0.4)
        logf = open(CAM_LOG, "a", encoding="utf-8")
        _camera_proc = subprocess.Popen(
            ["python3", "-u", str(CAM_SCRIPT)],
            stdout=logf,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f"[CAM] 已启动 pid={_camera_proc.pid}")
        return {"ok": True, "running": True, "message": "摄像头已启动", "pid": _camera_proc.pid}


def stop_camera() -> dict:
    global _camera_proc
    with _camera_lock:
        if _camera_proc is not None and _camera_proc.poll() is None:
            try:
                _camera_proc.terminate()
                _camera_proc.wait(timeout=3)
            except Exception:
                try:
                    _camera_proc.kill()
                except Exception:
                    pass
        _camera_proc = None
        subprocess.run(["pkill", "-f", "camera_stream.py"], capture_output=True)
        subprocess.run(["pkill", "-f", "ffmpeg.*video0"], capture_output=True)
        print("[CAM] 已停止")
        return {"ok": True, "running": False, "message": "摄像头已关闭"}


ZERO_PROFILES_FILE = Path(__file__).resolve().parent / "zero_profiles.json"
IMU_TCP_HOST = "127.0.0.1"
IMU_TCP_PORT = 8888
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

CONTROL_MODES = ("manual", "imu_hold", "hybrid")
individual_motor_active = False
individual_motor_values = [0.0] * 8
individual_motor_lock = threading.Lock()

raw_state = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
}

DEFAULT_ZERO_PROFILE = {
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
}


def load_zero_profiles() -> dict[str, dict[str, float]]:
    profiles = {
        name: dict(DEFAULT_ZERO_PROFILE)
        for name in ("A", "B", "C")
    }
    active = "A"
    if ZERO_PROFILES_FILE.is_file():
        try:
            data = json.loads(ZERO_PROFILES_FILE.read_text(encoding="utf-8"))
            for name in ("A", "B", "C"):
                if name in data.get("profiles", {}):
                    profiles[name].update(data["profiles"][name])
            active = data.get("active", "A")
            if active not in profiles:
                active = "A"
        except (json.JSONDecodeError, OSError, TypeError, ValueError) as exc:
            print(f"[WARN] 零位配置读取失败，使用默认值: {exc}")
    return profiles, active


def save_zero_profiles() -> None:
    payload = {
        "active": state["active_zero_profile"],
        "profiles": zero_profiles,
    }
    try:
        ZERO_PROFILES_FILE.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
    except OSError as exc:
        print(f"[WARN] 零位配置保存失败: {exc}")


zero_profiles, _active_profile = load_zero_profiles()

state: dict[str, object] = {
    "connected": False,
    "ax": 0.0, "ay": 0.0, "az": 0.0,
    "gx": 0.0, "gy": 0.0, "gz": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0,
    "active_zero_profile": _active_profile,
    "control_mode": "manual",
    "imu_cmd_roll": 0.0,
    "imu_cmd_pitch": 0.0,
    "imu_cmd_yaw": 0.0,
    "imu_cmd_heave": 0.0,
    "thruster_cmd_roll": 0.0,
    "thruster_cmd_pitch": 0.0,
    "thruster_cmd_yaw": 0.0,
    "thruster_cmd_heave": 0.0,
    "thruster_cmd_surge": 0.0,
    "spi_active": False,
    "spi_tx_count": 0,
    "spi_err_count": 0,
    "last_spi_channels": [0] * 8,
    "transport_mode": "spi",
    "stlink": {},
}

state_lock = threading.Lock()
imu_sock: socket.socket | None = None
imu_sock_lock = threading.Lock()

manual_cmd = {
    "heave": 0.0,
    "pitch": 0.0,
    "roll": 0.0,
    "surge": 0.0,
    "yaw": 0.0,
    "speed_mode": "medium",
}
manual_lock = threading.Lock()
last_manual_update = time.time()
MANUAL_WATCHDOG_SEC = 1.0  # 超过此时间无控制更新 → 急停（丢帧保护）

control_mode = "manual"
control_mode_lock = threading.Lock()

# 控制途径：spi = 通过 SPI 下发（默认）；stlink = 通过 ST-Link 直写 CCR（调试/硬件验证）。
transport_mode = "spi"
transport_lock = threading.Lock()


def get_transport() -> str:
    with transport_lock:
        return transport_mode


def set_transport(mode: str) -> None:
    global transport_mode
    with transport_lock:
        transport_mode = mode
    with state_lock:
        state["transport_mode"] = mode
    print(f"[INFO] 控制途径切换为: {mode}")

imu_controller = ImuThrusterController()
last_control_tick = time.time()

esc_instance = None
spi_tx_count = 0
spi_err_count = 0
last_spi_channels = [0] * 8
last_pushed_channels: list[int] | None = None
last_spi_send_time = 0.0
last_spi_ok_time = time.time()      # 初始化为当前时间，避免启动时看门狗误触发
CONTROL_LOOP_SEC = 0.025  # 40Hz
IDLE_KEEPALIVE_SEC = 1.0
SPI_LOCK_TIMEOUT_LOOP = 0.05   # 控制循环锁超时：宁可跳帧也不能卡死
SPI_LOCK_TIMEOUT_HTTP = 0.2    # HTTP 请求锁超时：给控制循环让路
SPI_WATCHDOG_SEC = 2.0         # 超过此时间无成功发送 → 告警并复位
DEBUG_LOG_SPI = False          # 设为 True 可打印每帧内容（调试用）

# ── SPI 状态监控与日志 ──
SPI_STATUS_LOG = Path("/tmp/spi_status.log")
SPI_STATUS_LOG_INTERVAL = 2.0        # 检测间隔
SPI_STATUS_SUMMARY_INTERVAL = 10.0   # 摘要间隔
_spi_prev_status: dict | None = None

# 供 SSE 前端实时展示的最近 SPI 事件
_spi_events: list[str] = []          # 最多保留 10 条
_spi_events_lock = threading.Lock()

def spi_status_monitor() -> None:
    """后台线程：周期性检测 SPI/STM32 通信状态，记录变化并写日志。"""
    global _spi_prev_status
    last_summary = 0.0
    while True:
        try:
            st = get_stm32_status()
            now = time.time()
            ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))

            # 检测状态变化
            if _spi_prev_status is not None:
                changes: list[str] = []
                if st["miso_ok"] != _spi_prev_status["miso_ok"]:
                    changes.append(f"MISO: {_spi_prev_status['miso_ok']} → {st['miso_ok']}")
                if st["stm32_timeout"] != _spi_prev_status["stm32_timeout"]:
                    changes.append(f"TIMEOUT: {_spi_prev_status['stm32_timeout']} → {st['stm32_timeout']}")
                if st["stm32_mode_index"] != _spi_prev_status["stm32_mode_index"]:
                    changes.append(f"mode: {_spi_prev_status['stm32_mode_index']} → {st['stm32_mode_index']}")
                fail_diff = st["stm32_frame_fail"] - _spi_prev_status["stm32_frame_fail"]
                if fail_diff > 0:
                    changes.append(f"FAIL +{fail_diff} (total {st['stm32_frame_fail']})")
                ok_diff = st["stm32_frame_ok"] - _spi_prev_status["stm32_frame_ok"]
                if ok_diff > 0:
                    changes.append(f"OK +{ok_diff} (total {st['stm32_frame_ok']})")

                if changes:
                    line = f"[{ts}] CHANGE: {'; '.join(changes)}"
                    with open(SPI_STATUS_LOG, "a", encoding="utf-8") as f:
                        f.write(line + "\n")
                    print(line)
                    with _spi_events_lock:
                        _spi_events.append(line)
                        if len(_spi_events) > 10:
                            _spi_events = _spi_events[-10:]

            # 定期摘要
            if now - last_summary >= SPI_STATUS_SUMMARY_INTERVAL:
                line = (f"[{ts}] SUMMARY: MISO={'OK' if st['miso_ok'] else 'N/A'} "
                        f"OK={st['stm32_frame_ok']} FAIL={st['stm32_frame_fail']} "
                        f"mode={st['stm32_mode_index']} "
                        f"timeout={'YES' if st['stm32_timeout'] else 'no'} "
                        f"PWM={st['stm32_pwm_update']} "
                        f"INT={st['stm32_interrupt_count']} "
                        f"TX={spi_tx_count} ERR={spi_err_count}")
                with open(SPI_STATUS_LOG, "a", encoding="utf-8") as f:
                    f.write(line + "\n")
                last_summary = now

            _spi_prev_status = st
        except Exception as exc:
            with open(SPI_STATUS_LOG, "a", encoding="utf-8") as f:
                f.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] ERROR: {exc}\n")
        time.sleep(SPI_STATUS_LOG_INTERVAL)

# SPI 总线互斥统一用 esc_spi.spi_bus_lock()：它同时做进程内串行与跨进程文件锁
# (/tmp/propeller_spi.lock)，与 spi_send_burst.py、esc_spi 的 TUI 共用同一把。
# 总线争用会让 STM32 收到交错帧、校验失败，固件每累计 200 次失败自动切一次 SPI
# 模式 → 与 Pi 固定 mode 0 永久错位、无法自愈。


# 逻辑通道 -> 物理通道 的置换：物理[i] = 逻辑[CHANNEL_WIRING[i]]。
# 接线为自然映射（通道 index i → 电机 i+1）：A8→M1, A9→M2, A10→M3, A11→M4,
# B6→M5, B7→M6, B8→M7, B9→M8（用户确认 A9 接电机 2）。故不做任何换位。
# 若日后确实发现某两路接反，只需在此表对调对应下标。
CHANNEL_WIRING = [0, 1, 2, 3, 4, 5, 6, 7]


def apply_wiring(channels: list[int]) -> list[int]:
    return [channels[i] for i in CHANNEL_WIRING]
if ESC_SPI:
    try:
        Path("/dev/spidev0.0").resolve()
        state["spi_active"] = True
        print("[INFO] SPI 设备就绪 (40Hz 控制环 + 指令变化立即下发)")
    except OSError as exc:
        state["spi_active"] = False
        print(f"[WARN] SPI 设备不可用: {exc}")


def spi_get() -> "ESC_SPI | None":
    """持久 SPI 连接，避免每次 open/close 增加 tens of ms 延迟。"""
    global esc_instance
    if ESC_SPI is None:
        return None
    if esc_instance is None:
        esc_instance = ESC_SPI(bus=0, device=0, max_speed_hz=SPI_SPEED_HZ, mode=0)
    return esc_instance


def spi_close() -> None:
    global esc_instance
    if esc_instance is not None:
        try:
            with spi_bus_lock(timeout=0.5):
                if esc_instance is not None:
                    esc_instance.close()
                    esc_instance = None
        except (TimeoutError, OSError):
            esc_instance = None  # 强制清空，防止死锁里的引用


def arm_escs_at_boot() -> None:
    """服务启动后先发 3s 中位 PWM，帮助电调从自检/掉电状态完成解锁。"""
    if ESC_SPI is None:
        return
    print("[INFO] 电调解锁：发送 3s 中位 PWM ...")
    for _ in range(30):
        spi_push_channels([0] * 8, burst=2, lock_timeout=SPI_LOCK_TIMEOUT_LOOP)
        time.sleep(0.1)
    print("[INFO] 电调解锁完成，可以控制")


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))


def compute_channels_now() -> list[int]:
    """按当前状态立即计算 8 路 SPI 通道值。"""
    speed_limits = {"slow": 25.0, "medium": 50.0, "fast": 80.0}
    with individual_motor_lock:
        if individual_motor_active:
            # 独立电机模式也套 MOTOR_SIGN：滑块正值统一表示「1-6 上浮 / 7-8 前进」，
            # 由 MOTOR_SIGN 抵消各桨的物理装反，操作者不用记哪台是反的。
            # ⚠ 因此本模式**不能再用来测量** MOTOR_SIGN（它已经把方向纠正掉了）；
            #   现在它是**验证**工具：八路都给正值，应当全部上浮/前进，
            #   哪一路不是，就是那一路的 MOTOR_SIGN 记错了。
            return [
                int(clamp(sgn * v, -100, 100))
                for sgn, v in zip(MOTOR_SIGN, individual_motor_values)
            ]
    with manual_lock:
        limit = speed_limits.get(manual_cmd["speed_mode"], 50.0)
    h, p, r, s, y = compute_thruster_commands()
    return mix_thrusters(h, p, r, s, y, limit)


def stlink_push_channels(channels: list[int]) -> bool:
    """STLink 途径：把 8 路百分比通道转微秒后，直接写 TIM CCR 寄存器。

    返回 True 表示写入成功。内置节流（stlink_control 内部 MIN_WRITE_INTERVAL_SEC），
    适合低频调试，不适合 40Hz 闭环。
    """
    global last_spi_channels, last_pushed_channels, last_spi_ok_time
    last_spi_channels = list(channels)
    phys_channels = apply_wiring(channels)
    phys_us = [percent_to_us(v, MOTOR_TRIM_US[i]) for i, v in enumerate(phys_channels)]
    ok = stlink_control.stlink_write_us(phys_us)
    if ok:
        last_pushed_channels = list(channels)
        last_spi_ok_time = time.time()
    with state_lock:
        state["last_spi_channels"] = list(channels)
        state["stlink"] = stlink_control.get_stlink_status()
        state["spi_active"] = ok
    return ok


def spi_push_channels(channels: list[int], *, burst: int = 1,
                     lock_timeout: float | None = None) -> bool:
    """向 STM32 下发 SPI 帧。持久连接 + 短帧间隔。

    返回 True 表示发送成功，False 表示被跳过（锁超时等）。
    lock_timeout=None 为阻塞；控制循环传 SPI_LOCK_TIMEOUT_LOOP。

    若当前控制途径为 stlink，则改走 ST-Link 直写 CCR。
    """
    global spi_tx_count, spi_err_count, last_spi_channels, last_spi_send_time, last_pushed_channels
    global last_spi_ok_time
    if get_transport() == "stlink":
        return stlink_push_channels(channels)
    if ESC_SPI is None:
        return False
    last_spi_channels = list(channels)          # 显示用：逻辑通道百分比（重映射前）
    phys_channels = apply_wiring(channels)      # 物理通道顺序（重映射后）
    phys_us = [percent_to_us(v, MOTOR_TRIM_US[i]) for i, v in enumerate(phys_channels)]
    ok = False
    try:
        with spi_bus_lock(timeout=lock_timeout):
            try:
                esc = spi_get()
                if esc is None:
                    return False
                esc.set_all(phys_us)
                for _ in range(max(1, burst)):
                    esc.send_frame()
                    if burst > 1:
                        time.sleep(SPI_FRAME_GAP_SEC)
                spi_tx_count += 1
                last_spi_send_time = time.time()
                last_spi_ok_time = last_spi_send_time
                last_pushed_channels = list(channels)
                ok = True
                if DEBUG_LOG_SPI:
                    print(f"[SPI] tx={spi_tx_count} ch={channels} us={phys_us}")
            except OSError as exc:
                ok = False
                spi_err_count += 1
                spi_close()
                if spi_err_count <= 5 or spi_err_count % 50 == 0:
                    print(f"[ERROR] SPI 发送失败 #{spi_err_count}: {exc}")
    except TimeoutError:
        # 锁超时：被其他进程占用，跳过本帧但不报错（下个循环会重试）
        if spi_err_count <= 3 or spi_err_count % 50 == 0:
            print(f"[WARN] SPI 锁超时，跳过本帧")
    with state_lock:
        state["spi_tx_count"] = spi_tx_count
        state["spi_err_count"] = spi_err_count
        state["last_spi_channels"] = list(channels)
        if ok:
            state["spi_active"] = True
    return ok


_last_http_push_time = 0.0

def push_control_spi_now(*, force: bool = False) -> None:
    """HTTP 控制请求到达后立即混控并下发 SPI（不等待后台线程）。

    内置节流：同一帧内容不重复发送；连续请求间隔 < 10ms 则跳过。
    force=True 绕过节流（紧急停止用）。
    """
    global _last_http_push_time
    now = time.time()
    if not force and now - _last_http_push_time < 0.01:
        return  # 节流：10ms 内最多发一次
    _last_http_push_time = now

    channels = compute_channels_now()
    prev = last_pushed_channels
    if force or prev is None or prev != channels:
        lock_to = None if force else SPI_LOCK_TIMEOUT_HTTP  # 紧急停止用阻塞锁
        spi_push_channels(channels, burst=1, lock_timeout=lock_to)


def compute_thruster_commands() -> tuple[float, float, float, float, float]:
    """根据控制模式，融合 IMU 反馈与手动指令，输出 h,p,r,s,y。

    更新式控制：每次收到新指令直接取缔旧值；
    短时丢帧维持最后值，超 MANUAL_WATCHDOG_SEC 自动急停。
    """
    global last_control_tick

    now = time.time()
    dt = clamp(now - last_control_tick, 0.001, 0.1)
    last_control_tick = now

    with control_mode_lock:
        mode = control_mode

    with state_lock:
        imu_connected = bool(state["connected"])
        imu = {
            "roll": float(state["roll"]),
            "pitch": float(state["pitch"]),
            "yaw": float(state["yaw"]),
            "gx": float(state["gx"]),
            "gy": float(state["gy"]),
            "gz": float(state["gz"]),
            "az": float(state["az"]),
            "pos_z": float(state["pos_z"]),
        }

    with manual_lock:
        manual = dict(manual_cmd)
        manual_age = now - last_manual_update

    # 更新式：直接使用最新指令值
    h = manual["heave"]
    p = manual["pitch"]
    r = manual["roll"]
    s = manual["surge"]
    y = manual["yaw"]

    imu_cmd = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "heave": 0.0}

    if mode == "imu_hold" and imu_connected:
        # IMU 保持模式：长时间丢帧也急停
        if manual_age > MANUAL_WATCHDOG_SEC:
            h = p = r = s = y = 0.0
        else:
            imu_controller.set_targets(0.0, 0.0, 0.0, 0.0)
            imu_cmd = imu_controller.compute(imu, dt, hold_depth=False)
            p = imu_cmd["pitch"]
            r = imu_cmd["roll"]
            y = imu_cmd["yaw"]
            h = manual["heave"]

    elif mode == "hybrid" and imu_connected:
        if manual_age > MANUAL_WATCHDOG_SEC:
            h = p = r = s = y = 0.0
        else:
            imu_controller.set_targets(
                roll=manual["roll"] * 30.0,
                pitch=manual["pitch"] * 30.0,
                yaw=manual["yaw"] * 45.0,
                heave=manual["heave"] * 0.5,
            )
            imu_cmd = imu_controller.compute(imu, dt, hold_depth=abs(manual["heave"]) < 0.05)
            p = imu_cmd["pitch"]
            r = imu_cmd["roll"]
            y = imu_cmd["yaw"]
            h = imu_cmd["heave"] if abs(manual["heave"]) < 0.05 else manual["heave"]
            s = manual["surge"]

    elif mode == "manual":
        imu_controller.reset()
        # 丢帧看门狗：超过阈值 → 急停（全零）
        if manual_age > MANUAL_WATCHDOG_SEC:
            h = p = r = s = y = 0.0

    with state_lock:
        state["control_mode"] = mode
        state["imu_cmd_roll"] = imu_cmd["roll"]
        state["imu_cmd_pitch"] = imu_cmd["pitch"]
        state["imu_cmd_yaw"] = imu_cmd["yaw"]
        state["imu_cmd_heave"] = imu_cmd["heave"]
        state["thruster_cmd_roll"] = r
        state["thruster_cmd_pitch"] = p
        state["thruster_cmd_yaw"] = y
        state["thruster_cmd_heave"] = h
        state["thruster_cmd_surge"] = s

    return h, p, r, s, y


def update_thrusters() -> None:
    """混控计算 + SPI 下发：40Hz；通道变化或松键归零立即发送。"""
    global last_pushed_channels, last_spi_ok_time
    while True:
        channels = compute_channels_now()

        with individual_motor_lock:
            indi_active = individual_motor_active
            indi_values = list(individual_motor_values)
        with state_lock:
            state["individual_motor_active"] = indi_active
            state["individual_motor_values"] = indi_values
            state["last_spi_channels"] = list(channels)

        prev = last_pushed_channels
        changed = prev is None or prev != channels
        now = time.time()
        moving = any(ch != 0 for ch in channels)
        if changed:
            spi_push_channels(channels, burst=1, lock_timeout=SPI_LOCK_TIMEOUT_LOOP)
        elif moving:
            spi_push_channels(channels, burst=1, lock_timeout=SPI_LOCK_TIMEOUT_LOOP)
        elif now - last_spi_send_time >= IDLE_KEEPALIVE_SEC:
            spi_push_channels([0] * 8, burst=1, lock_timeout=SPI_LOCK_TIMEOUT_LOOP)

        # SPI 看门狗：超过阈值无成功发送 → 告警并尝试恢复
        # （仅 SPI 途径生效；STLink 途径的写入由 stlink_control 内部节流与状态管理）
        if get_transport() == "spi" and now - last_spi_ok_time > SPI_WATCHDOG_SEC and last_spi_ok_time > 0:
            print(f"[WARN] SPI 看门狗触发：{now - last_spi_ok_time:.1f}s 无成功发送，复位 SPI 连接")
            spi_close()
            last_spi_ok_time = now  # 防止重复告警

        time.sleep(CONTROL_LOOP_SEC)


def update_state_from_line(line: str) -> None:
    parts = line.strip().split(",")
    if parts[0] != "IMU":
        return

    try:
        values = [
            0.0 if math.isnan(float(v)) or math.isinf(float(v)) else float(v)
            for v in parts[1:]
        ]
    except ValueError:
        return

    with state_lock:
        state["connected"] = True
        if len(values) >= 9:
            state["ax"], state["ay"], state["az"] = values[0:3]
            state["gx"], state["gy"], state["gz"] = values[3:6]
            raw_state["roll"], raw_state["pitch"], raw_state["yaw"] = values[6:9]
        if len(values) >= 12:
            raw_state["pos_x"], raw_state["pos_y"], raw_state["pos_z"] = values[9:12]

        prof = zero_profiles[state["active_zero_profile"]]
        state["roll"] = raw_state["roll"] - prof["roll"]
        state["pitch"] = raw_state["pitch"] - prof["pitch"]
        state["yaw"] = wrap_angle_deg(raw_state["yaw"] - prof["yaw"])
        state["pos_x"] = raw_state["pos_x"] - prof["pos_x"]
        state["pos_y"] = raw_state["pos_y"] - prof["pos_y"]
        state["pos_z"] = raw_state["pos_z"] - prof["pos_z"]


def send_hardware_zero() -> bool:
    """向 IMU 驱动发送硬件置零命令（加速度/角速度/姿态角）。"""
    with imu_sock_lock:
        if imu_sock is None:
            return False
        try:
            imu_sock.sendall(b"ZERO\n")
            return True
        except OSError as exc:
            print(f"[WARN] 硬件置零命令发送失败: {exc}")
            return False


def resolve_static_path(url_path: str) -> Path | None:
    rel = url_path.lstrip("/")
    if rel.startswith("static/"):
        rel = rel[len("static/"):]
    if not rel or ".." in Path(rel).parts:
        return None
    file_path = (STATIC_DIR / rel).resolve()
    if not file_path.is_relative_to(STATIC_DIR.resolve()):
        return None
    return file_path


def imu_reader_loop() -> None:
    global imu_sock
    while True:
        try:
            sock = socket.create_connection((IMU_TCP_HOST, IMU_TCP_PORT), timeout=5)
            sock.settimeout(1.0)
            with imu_sock_lock:
                imu_sock = sock
            print(f"[INFO] 已连接 IMU 驱动 {IMU_TCP_HOST}:{IMU_TCP_PORT}")

            buffer = ""
            while True:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if line.startswith("IMU,"):
                        update_state_from_line(line)
        except OSError as exc:
            with state_lock:
                state["connected"] = False
            with imu_sock_lock:
                if imu_sock is not None:
                    try:
                        imu_sock.close()
                    except OSError:
                        pass
                    imu_sock = None
            print(f"[WARN] 无法连接 IMU 驱动: {exc}")
            time.sleep(2)


class ImuWebHandler(BaseHTTPRequestHandler):
    server_version = "RobotWebServer/3.0"
    # 保持默认 HTTP/1.0：SSE /stream 无 Content-Length/分块编码，若开 HTTP/1.1 keep-alive
    # 会让浏览器 EventSource 无法确定消息边界 -> 面板显示“断开”。短连接 + 下面的 TCP_NODELAY
    # 在网线直连下延迟已足够低。

    def setup(self) -> None:
        super().setup()
        # 关闭 Nagle：小体积控制包立即发出，去掉最多 ~40ms 的合并等待。
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    def log_message(self, format: str, *args: object) -> None:
        if self.path.startswith(("/stream", "/api/control")):
            return
        print(f"[HTTP] {self.address_string()} {format % args}")

    def _read_json_body(self) -> dict | None:
        length = int(self.headers.get("Content-Length", 0))
        if length <= 0:
            return None
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None

    def _send_json(self, payload: dict[str, object], status: int = 200) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, file_path: Path) -> None:
        if not file_path.is_file():
            self.send_error(404, "File not found")
            return
        content = file_path.read_bytes()
        content_type, _ = mimetypes.guess_type(str(file_path))
        self.send_response(200)
        self.send_header("Content-Type", content_type or "application/octet-stream")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]

        if path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    with state_lock:
                        s = dict(state)
                    s["stm32"] = get_stm32_status()
                    s["stlink"] = stlink_control.get_stlink_status()
                    with _spi_events_lock:
                        s["spi_events"] = list(_spi_events)
                    payload = json.dumps(s, ensure_ascii=False)
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return

        if path in ("/api/data", "/api/status"):
            with state_lock:
                s = dict(state)
            s["stm32"] = get_stm32_status()
            s["stlink"] = stlink_control.get_stlink_status()
            with _spi_events_lock:
                s["spi_events"] = list(_spi_events)
            self._send_json(s)
            return

        if path == "/api/camera/status":
            self._send_json(_camera_status())
            return

        if path == "/":
            self._send_file(STATIC_DIR / "index.html")
            return

        static_path = resolve_static_path(path)
        if static_path is not None:
            self._send_file(static_path)
            return

        self.send_error(404, "Not found")

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]

        if path == "/api/control":
            data = self._read_json_body()
            if data is None:
                self.send_error(400, "Bad Request")
                return
            with manual_lock:
                global last_manual_update
                for key in ("heave", "pitch", "roll", "surge", "yaw"):
                    if key in data:
                        manual_cmd[key] = float(data[key])
                if "speed_mode" in data:
                    manual_cmd["speed_mode"] = str(data["speed_mode"])
                last_manual_update = time.time()
            # 键盘/摇杆一介入就自动退出独立电机模式
            any_input = any(
                abs(float(data.get(k, 0.0))) > 0.01
                for k in ("heave", "pitch", "roll", "surge", "yaw")
            )
            if any_input:
                with individual_motor_lock:
                    global individual_motor_active
                    if individual_motor_active:
                        individual_motor_active = False
                        print("[INFO] 手动控制介入，自动退出独立电机模式")
            push_control_spi_now()
            if DEBUG_LOG_SPI:
                with manual_lock:
                    _h = manual_cmd["heave"]
                    _s = manual_cmd["surge"]
                print(f"[CTRL] h={_h:.1f} s={_s:.1f} ch={compute_channels_now()}")
            self._send_json({"ok": True})
            return

        if path == "/api/control/mode":
            data = self._read_json_body()
            if not data or "mode" not in data:
                self.send_error(400, "Bad Request")
                return
            mode = str(data["mode"])
            if mode not in CONTROL_MODES:
                self.send_error(400, "Invalid mode")
                return
            with control_mode_lock:
                global control_mode
                control_mode = mode
            imu_controller.reset()
            print(f"[INFO] 控制模式切换为: {mode}")
            self._send_json({"ok": True, "mode": mode})
            return

        if path == "/api/transport":
            data = self._read_json_body()
            if not data or "mode" not in data:
                self.send_error(400, "Bad Request")
                return
            mode = str(data["mode"])
            if mode not in ("spi", "stlink"):
                self.send_error(400, "Invalid transport")
                return
            # 切换前先通过当前途径把油门收回中位，避免切换瞬间电机悬空
            spi_push_channels([0] * 8, burst=1, lock_timeout=SPI_LOCK_TIMEOUT_HTTP)
            set_transport(mode)
            self._send_json({
                "ok": True,
                "transport": mode,
                "stlink": stlink_control.get_stlink_status(),
            })
            return

        if path == "/api/emergency":
            with manual_lock:
                for key in ("heave", "pitch", "roll", "surge", "yaw"):
                    manual_cmd[key] = 0.0
                last_manual_update = time.time()
            push_control_spi_now(force=True)
            imu_controller.reset()
            print("[INFO] 紧急停止已触发")
            self._send_json({"ok": True, "message": "紧急停止已执行"})
            return

        if path == "/api/zero/set":
            data = self._read_json_body()
            if not data:
                self.send_error(400, "Bad Request")
                return
            profile = data.get("profile", "A")
            if profile in zero_profiles:
                with state_lock:
                    zero_profiles[profile]["roll"] = raw_state["roll"]
                    zero_profiles[profile]["pitch"] = raw_state["pitch"]
                    zero_profiles[profile]["yaw"] = raw_state["yaw"]
                    zero_profiles[profile]["pos_x"] = raw_state["pos_x"]
                    zero_profiles[profile]["pos_y"] = raw_state["pos_y"]
                    zero_profiles[profile]["pos_z"] = raw_state["pos_z"]
                save_zero_profiles()
                hw_ok = send_hardware_zero()
                imu_controller.reset()
                msg = f"已设置零位 {profile}"
                if hw_ok:
                    msg += "（含硬件置零）"
                self._send_json({"ok": True, "message": msg, "hardware_zero": hw_ok})
                return
            self.send_error(400, "Bad Request")
            return

        if path == "/api/zero/use":
            data = self._read_json_body()
            if not data:
                self.send_error(400, "Bad Request")
                return
            profile = data.get("profile", "A")
            if profile in zero_profiles:
                with state_lock:
                    state["active_zero_profile"] = profile
                save_zero_profiles()
                imu_controller.reset()
                self._send_json({"ok": True, "message": f"已切换零位 {profile}"})
                return
            self.send_error(400, "Bad Request")
            return

        if path == "/api/camera/start":
            self._send_json(start_camera())
            return

        if path == "/api/camera/stop":
            self._send_json(stop_camera())
            return

        if path == "/api/motor/individual":
            data = self._read_json_body()
            if data is None:
                self.send_error(400, "Bad Request")
                return
            with individual_motor_lock:
                global individual_motor_values  # individual_motor_active 已在上方声明为 global
                if "active" in data:
                    individual_motor_active = bool(data["active"])
                if "channels" in data:
                    ch = data["channels"]
                    if len(ch) == 8:
                        individual_motor_values = [float(v) for v in ch]
            push_control_spi_now()
            self._send_json({"ok": True, "active": individual_motor_active})
            return

        self.send_error(404, "Not found")


ETH_DIRECT_IP = "192.168.50.1"


def get_local_ip() -> str:
    """优先返回网线直连地址，便于笔记本通过 eth 访问。"""
    try:
        with open("/sys/class/net/eth0/operstate", encoding="utf-8") as f:
            if f.read().strip() in ("up", "unknown"):
                out = subprocess.check_output(
                    ["ip", "-4", "-o", "addr", "show", "eth0"],
                    text=True,
                    stderr=subprocess.DEVNULL,
                )
                for token in out.split():
                    if token.startswith(f"{ETH_DIRECT_IP}/"):
                        return ETH_DIRECT_IP
    except (OSError, subprocess.CalledProcessError):
        pass

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return ETH_DIRECT_IP


_shutdown_once = threading.Event()


def stop_thrusters_and_close(reason: str = "") -> None:
    """停机兜底：把 8 路推进器收回中位，然后关闭 SPI。

    ★ 必须做，因为 STM32 侧没有失控保护：固件的 ESC_TimeoutHandler() 从未被调用
      （App.c 里 Millisecond_Task / Millisecond_50_Task 都是空的），Pi 一旦停发，
      最后一次油门会被 PWM 永久保持下去，电机不会自己停。
    """
    if _shutdown_once.is_set():
        return
    _shutdown_once.set()
    try:
        print(f"[INFO] 停机兜底：下发中位停机帧 {reason}".rstrip(), flush=True)
        spi_push_channels([0] * 8, burst=5)  # 关机路径：阻塞式确保停机帧发出
    except Exception as exc:  # 兜底路径不允许再抛出
        print(f"[ERROR] 停机兜底发送失败: {exc}", flush=True)
    finally:
        spi_close()


def _handle_stop_signal(signum, _frame):
    """SIGTERM/SIGINT：抛 SystemExit 让主线程走 main() 的 finally 兜底。

    守护脚本 stop 时发的是 SIGTERM，Python 默认会直接终止、finally 不执行 ——
    那样电机会保持最后油门继续转，所以这里必须显式接管。
    """
    print(f"\n[INFO] 收到 {signal.Signals(signum).name}，准备停机", flush=True)
    raise SystemExit(0)


def main() -> None:
    if not STATIC_DIR.is_dir():
        raise SystemExit(f"缺少网页目录: {STATIC_DIR}")

    signal.signal(signal.SIGTERM, _handle_stop_signal)
    signal.signal(signal.SIGINT, _handle_stop_signal)

    threading.Thread(target=imu_reader_loop, daemon=True).start()
    threading.Thread(target=arm_escs_at_boot, daemon=True).start()
    threading.Thread(target=update_thrusters, daemon=True).start()
    threading.Thread(target=spi_status_monitor, daemon=True).start()

    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), ImuWebHandler)
    local_ip = get_local_ip()
    _stlink_now = stlink_control.get_stlink_status()
    print("=" * 56)
    print("  水下机器人 IMU 闭环控制服务已启动")
    print(f"  网线直连: http://{ETH_DIRECT_IP}:{WEB_PORT}")
    print(f"  当前可用: http://{local_ip}:{WEB_PORT}")
    print(f"  SPI/STM32: {'已连接' if state.get('spi_active') else '模拟模式'}")
    print(f"  ST-Link: {'已连接' if _stlink_now.get('device_present') else '未检测到'}")
    print("  控制模式: manual / imu_hold / hybrid")
    print("  控制途径: spi（SPI 下发） / stlink（ST-Link 直写 CCR）")
    print("=" * 56)

    try:
        server.serve_forever()
    except (KeyboardInterrupt, SystemExit):
        print("\n[INFO] 服务已停止")
    finally:
        server.server_close()
        stop_thrusters_and_close("(服务退出)")


if __name__ == "__main__":
    main()
