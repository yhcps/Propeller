#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 ST-Link (OpenOCD) 直接读写 8 路推进器 PWM 的 CCR 寄存器。

这是 stlink_pwm.sh 的 Python 封装，供 web 服务在「STLink 控制途径」下调用。
与 SPI 途径不同，STLink 途径绕过 STM32 的 SPI 从机，直接写 TIM1/TIM4 的
比较寄存器，主要用于硬件验证（排查接线 / 电调 / 个别通道不转等问题）。

⚠ 注意事项：
  * openocd 每次 `init` 会连接并 halt CPU；写完后 `shutdown` 恢复运行。
    固件主循环不重写 CCR，因此写入值会保持，直到下一次 SPI 帧覆盖它。
  * 单次 openocd 调用约 1~2s，远慢于 SPI 的 40Hz，故 STLink 途径只适合
    低频 / 单次调试，不应作为常规闭环控制。
  * sudo 密码与 stlink_pwm.sh 保持一致（内部工具，仅本地使用）。
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import time

# TIM1/TIM4 CCR 寄存器地址（与 stlink_pwm.sh 一致）
# TIM1 base 0x40012C00, TIM4 base 0x40000800, CCR1..4 offset 0x34,0x38,0x3C,0x40
CCR_ADDR = (
    0x40012C34,  # ch0 TIM1_CH1 → PA8
    0x40012C38,  # ch1 TIM1_CH2 → PA9
    0x40012C3C,  # ch2 TIM1_CH3 → PA10
    0x40012C40,  # ch3 TIM1_CH4 → PA11
    0x40000834,  # ch4 TIM4_CH1 → PB6
    0x40000838,  # ch5 TIM4_CH2 → PB7
    0x4000083C,  # ch6 TIM4_CH3 → PB8
    0x40000840,  # ch7 TIM4_CH4 → PB9
)

CH_GPIO = ("PA8", "PA9", "PA10", "PA11", "PB6", "PB7", "PB8", "PB9")

OPENOCD_INTERFACE = "/usr/share/openocd/scripts/interface/stlink.cfg"
OPENOCD_TARGET = "/usr/share/openocd/scripts/target/stm32f1x.cfg"

# 与 stlink_pwm.sh 的 SUDO_PASS 保持一致；可用环境变量覆盖。
SUDO_PASS = os.environ.get("STLINK_SUDO_PASS", "200655")

# STLink 途径的最小写间隔（秒）：openocd 每次 init 约 1~2s，高频调用会拖垮控制循环。
MIN_WRITE_INTERVAL_SEC = 0.5

_last_write_time = 0.0
_last_status: dict = {
    "connected": False,
    "last_ok": False,
    "last_error": "",
    "last_write_time": 0.0,
    "write_count": 0,
}


def stlink_detect() -> bool:
    """检测 ST-Link 是否插在 USB 上。仅判断设备存在，不做 openocd 连接。"""
    if shutil.which("openocd") is None:
        return False
    try:
        out = subprocess.run(
            ["lsusb"], capture_output=True, text=True, timeout=5
        ).stdout.lower()
        return ("stlink" in out) or ("0483" in out and "3748" in out)
    except (OSError, subprocess.SubprocessError):
        return False


def _run_openocd(commands: list[str], timeout: float = 15.0) -> tuple[str, str, int]:
    """执行一次 openocd 会话，返回 (stdout, stderr, returncode)。

    commands 是在 `init` 之后依次执行的 -c 参数；会话末尾自动 shutdown。
    """
    cmd = [
        "sudo", "-S", "openocd",
        "-f", OPENOCD_INTERFACE,
        "-f", OPENOCD_TARGET,
        "-c", "adapter speed 100",
        "-c", "transport select hla_swd",
        "-c", "init",
    ]
    for c in commands:
        cmd += ["-c", c]
    cmd += ["-c", "shutdown"]

    proc = subprocess.run(
        cmd,
        input=SUDO_PASS + "\n",
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return proc.stdout, proc.stderr, proc.returncode


def stlink_write_us(channels_us: list[int], *, force: bool = False) -> bool:
    """一次 openocd 会话写入 8 路 CCR（单位：微秒，1000~2000）。

    channels_us 长度必须为 8。内置最小写间隔节流，避免 40Hz 控制循环疯狂起
    openocd 进程；force=True 绕过节流（停机兜底用）。
    """
    global _last_write_time
    if len(channels_us) != 8:
        _last_status.update(connected=False, last_ok=False,
                            last_error=f"通道数错误: {len(channels_us)}")
        return False

    now = time.time()
    if not force and now - _last_write_time < MIN_WRITE_INTERVAL_SEC:
        return _last_status.get("last_ok", False)

    cmds = []
    for i in range(8):
        us = int(channels_us[i])
        if not (900 <= us <= 2100):
            _last_status.update(connected=False, last_ok=False,
                                last_error=f"通道 {i} 脉宽 {us} 越界")
            return False
        cmds.append(f"mww 0x{CCR_ADDR[i]:08X} {us}")

    try:
        _stdout, stderr, code = _run_openocd(cmds)
    except (OSError, subprocess.SubprocessError, subprocess.TimeoutExpired) as exc:
        _last_status.update(connected=False, last_ok=False, last_error=str(exc))
        return False

    ok = code == 0
    _last_write_time = now
    _last_status.update(
        connected=ok,
        last_ok=ok,
        last_error="" if ok else stderr.strip().splitlines()[-1] if stderr.strip() else f"openocd exit {code}",
        last_write_time=now,
        write_count=_last_status.get("write_count", 0) + (1 if ok else 0),
    )
    return ok


def stlink_stop() -> bool:
    """全部 8 路回中位 1500us。"""
    return stlink_write_us([1500] * 8, force=True)


def _parse_mdw_value(line: str) -> int | None:
    """解析 openocd `mdw 0xADDR 1` 输出，如 `0x40012c34: 000005dc`。"""
    m = re.search(r"0x[0-9a-fA-F]+:\s*([0-9a-fA-F]+)", line)
    if not m:
        return None
    return int(m.group(1), 16)


def stlink_read_us() -> list[int] | None:
    """读取 8 路 CCR 当前值（微秒）。失败返回 None。"""
    cmds = [f"mdw 0x{CCR_ADDR[i]:08X} 1" for i in range(8)]
    try:
        stdout, stderr, code = _run_openocd(cmds, timeout=20.0)
    except (OSError, subprocess.SubprocessError, subprocess.TimeoutExpired):
        return None
    if code != 0:
        return None
    values: list[int] = []
    for line in (stdout + stderr).splitlines():
        v = _parse_mdw_value(line)
        if v is not None:
            values.append(v)
    return values if len(values) == 8 else None


def get_stlink_status() -> dict:
    """返回 STLink 途径的连接状态快照，供 SSE / API 展示。"""
    st = dict(_last_status)
    # 每次查询时实时刷新「设备是否插着」这一项（轻量，不碰 openocd）
    st["device_present"] = stlink_detect()
    return st
