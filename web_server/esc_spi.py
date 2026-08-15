#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""树莓派 → STM32F103C8T6 的 8 路 ESC 油门 SPI 下发（全双工，带 MISO 状态回读）。

═══════════════ 固件事实（已烧录、不可改动，本模块据此适配）═══════════════
来源：STM32 工程 My_Core/{spi_slave.c, esc_spi.c} + App.c，详见 docs/STM32固件SPI约束.md

  * STM32 = SPI2 从机；**mode 0**(CPOL=0,CPHA=0)；MSB first；8 bit；硬件 NSS(PB12 低有效)。
  * 全双工模式：MOSI 下发 26 字节控制帧，MISO 同时回读 26 字节状态帧。
  * 固件预挂"恰好 26 字节"的中断接收，收满即校验；`spi_validate_frame` **只认帧头在第 0 字节**，
    重对齐视图 build_aligned_view() 仅供调试，不进 ESC 更新路径。
      >>> 所以 Pi 必须"一次 xfer2 恰好一帧 26 字节"，帧间留 ≥数百 µs 间隔。<<<
  * 校验失败每累计 200 次，固件自动切一次 CPOL/CPHA(0→1→2→3) 且**无法自愈**，只能复位 STM32。
      >>> 所以任何总线争用都是致命的：所有发送方必须共用 spi_bus_lock()。<<<
  * 固件 ESC_TimeoutHandler() 从未被调用（App.c 的周期任务是空的）→ **STM32 侧没有失控保护**，
    Pi 一停发，最后一次油门永久保持。停机前必须由 Pi 主动发中位帧。

帧格式（固定 26 字节）：
    [0]      0xAA  帧头
    [1]      0x01  命令（写 8 路通道）
    [2]      16    数据长度 = 8 通道 × 2 字节
    [3..18]  8 × int16_t 小端
    [19..24] 6 × 0x00 填充
    [25]     CRC8（多项式 0x07，初值 0，无反射无异或，覆盖前 25 字节）

MISO 回读状态帧（26 字节）：
    [0]     0xAA          帧头
    [1]     0x02          状态回传命令
    [2]     16            DATA_LEN
    [3..4]  uint16 LE     spi_validate_ok_count (低 16 位)
    [5..6]  uint16 LE     spi_validate_fail_count (低 16 位)
    [7..8]  int16 LE      throttle[0] 回读
    [9..10] int16 LE      throttle[1] 回读
    [11..12] uint16 LE    spi_interrupt_count (低 16 位)
    [13..14] uint16 LE    标志位: bit0=frame_valid, bit1=crc_ok, bit8=timeout_active
    [15..16] uint16 LE    spi_mode_index
    [17..18] uint16 LE    spi_pwm_update_count (低 16 位)
    [19..24] 6×0x00      填充
    [25]     CRC8

通道值语义（固件 ESC_ApplyToPWM）：
    -100 ≤ v ≤ 100 → 百分比，us = 1500 + v*5（-100→1000us，0→1500us 停，+100→2000us）
    其它值          → 直接当微秒脉宽
    最终一律钳位到 1000..2000us
  ⚠ 因此 101..999 这类中间值会被固件钳到 1000us（对双向电调 = 全速反转），本模块直接拒绝。
"""

from __future__ import annotations

import argparse
import fcntl
import os
import struct
import sys
import threading
import time
from contextlib import contextmanager
from pathlib import Path

try:
    import spidev
except ImportError:
    spidev = None


# ───────────────────────── SPI 参数（全项目唯一来源）─────────────────────────
# 固件只验证过 100k–500k。1MHz 时每字节仅 8µs，而 STM32 HAL 是逐字节中断，
# 容易溢出丢字节 → 帧失步 → 触发固件"200 次失败自动切模式"。
SPI_SPEED_HZ = 200_000
SPI_SPEED_MIN_HZ = 100_000
SPI_SPEED_MAX_HZ = 500_000
SPI_MODE = 0
SPI_BITS_PER_WORD = 8

#: 连发时的帧间间隔，保证固件那 26 字节的接收窗口能干净地收尾再重挂。
FRAME_GAP_SEC = 0.001

#: 每**物理通道**的中位微调（µs）。索引 = CHANNEL_WIRING 重映射**之后**的通道号，
#: 因为这是电调本身的属性，不随电机编号走。
#:
#: 油门 0 时脉宽 = 1500 + trim。当前全部为 0 → 标准中位 1500µs。
#: 若某电调中位偏离导致爬行，可用网页独立电机模式测死区中心后再写入偏置。
MOTOR_TRIM_US = (0, 0, 0, 0, 0, 0, 0, 0)


def percent_to_us(pct, trim_us: int = 0) -> int:
    """百分比油门(-100..100) → 直接微秒脉宽，并叠加该通道的中位微调。

    固件对 -100..100 当百分比处理（us = 1500 + v*5），对其它值当直接微秒并钳到
    1000..2000。这里统一改走**直接微秒**下发，好处是中位偏置能有 1µs 精度
    （百分比一档就是 5µs，压不出偏置）。

    `pct=0, trim_us=0` 时结果正好是 1500，与原来的百分比路径完全等价。
    """
    return max(1000, min(2000, 1500 + int(trim_us) + int(pct) * 5))

#: 跨进程 SPI 总线锁。守护进程与任何测试脚本共用同一把，防止帧交错。
SPI_LOCK_FILE = Path("/tmp/propeller_spi.lock")


# ───────────────────────── 总线互斥 ─────────────────────────
_lock_fh = None
_lock_depth = 0
_lock_guard = threading.RLock()  # 进程内串行；RLock 使同线程嵌套获取不死锁


@contextmanager
def spi_bus_lock(timeout: float | None = None):
    """独占 SPI 总线（进程内 + 跨进程）。

    所有向 /dev/spidev0.0 发帧的代码都必须包在这里面。历史教训：守护进程与测试
    脚本同时占用总线 → STM32 收到交错帧 → 大量校验失败 → 固件每 200 次失败自动
    切一次 SPI 模式 → 与 Pi 固定的 mode 0 永久错位，全部电机不转且无法自愈。

    可重入：同一进程内嵌套获取只在最外层真正上锁/解锁，避免自锁。

    timeout: 若指定（秒），在超时内无法获取锁则抛出 TimeoutError。
             控制循环应传 timeout=0.05 来避免长期阻塞；HTTP 请求可传 timeout=0.1。
    """
    global _lock_fh, _lock_depth
    with _lock_guard:
        if _lock_depth == 0:
            if _lock_fh is None:
                SPI_LOCK_FILE.touch(exist_ok=True)
                _lock_fh = SPI_LOCK_FILE.open("w")
            if timeout is None:
                fcntl.flock(_lock_fh, fcntl.LOCK_EX)
            else:
                deadline = time.monotonic() + timeout
                while True:
                    try:
                        fcntl.flock(_lock_fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
                        break
                    except BlockingIOError:
                        if time.monotonic() >= deadline:
                            raise TimeoutError(
                                f"SPI 总线锁获取超时 ({timeout:.2f}s)，可能有其他进程占用"
                            )
                        time.sleep(0.001)
        _lock_depth += 1
        try:
            yield
        finally:
            _lock_depth -= 1
            if _lock_depth == 0:
                fcntl.flock(_lock_fh, fcntl.LOCK_UN)


# ───────────────────────── 帧构造 ─────────────────────────
def crc8(data) -> int:
    """计算 CRC8，多项式 0x07，初始值 0，无反射、无异或。与固件 crc8() 一致。"""
    crc = 0
    for b in data:
        crc ^= b & 0xFF
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ──────────────── STM32 MISO 状态回读 ────────────────
# 由 send_frame() 在每次 SPI 传输后更新，imu_web_server 通过 get_stm32_status() 读取
_stm32_status = {
    "miso_ok": False,           # 最近一次 MISO 是否解析成功
    "stm32_frame_ok": 0,        # STM32 帧校验成功次数
    "stm32_frame_fail": 0,      # STM32 帧校验失败次数
    "stm32_interrupt_count": 0, # STM32 SPI 中断计数
    "stm32_pwm_update": 0,      # STM32 PWM 更新次数
    "stm32_mode_index": 0,      # STM32 SPI 模式索引 (0=CPOL0/CPHA0)
    "stm32_timeout": False,     # STM32 超时标志
    "stm32_flags": 0,           # 原始标志位
    "throttle_readback": [0] * 2,  # 油门[0..1] 回读
}


def _le16(data: bytes, offset: int) -> int:
    """小端 uint16 解码，带符号扩展（int16→int）。"""
    raw = data[offset] | (data[offset + 1] << 8)
    if raw & 0x8000:
        return raw - 0x10000
    return raw


def parse_stm32_miso(data: bytes):
    """解析 26 字节 MISO 状态帧，更新 _stm32_status。返回 True 表示成功。"""
    if len(data) < 26:
        _stm32_status["miso_ok"] = False
        return False
    if data[0] != 0xAA or data[1] != 0x02:
        _stm32_status["miso_ok"] = False
        return False
    if crc8(data[:25]) != data[25]:
        _stm32_status["miso_ok"] = False
        return False

    _stm32_status["miso_ok"] = True
    _stm32_status["stm32_frame_ok"] = data[3] | (data[4] << 8)
    _stm32_status["stm32_frame_fail"] = data[5] | (data[6] << 8)
    _stm32_status["throttle_readback"] = [_le16(data, 7), _le16(data, 9)]
    _stm32_status["stm32_interrupt_count"] = data[11] | (data[12] << 8)
    _stm32_status["stm32_flags"] = data[13] | (data[14] << 8)
    _stm32_status["stm32_mode_index"] = data[15] | (data[16] << 8)
    _stm32_status["stm32_pwm_update"] = data[17] | (data[18] << 8)
    _stm32_status["stm32_timeout"] = bool(_stm32_status["stm32_flags"] & 0x100)
    return True


def get_stm32_status():
    """返回 STM32 MISO 回读状态的快照。"""
    return dict(_stm32_status)


def validate_channel_value(value) -> int:
    """校验单通道取值，返回 int。

    固件只有两种合法解释：百分比 -100..100，或直接微秒 1000..2000。
    落在中间的值（如 500）会被固件钳到 1000us —— 对双向电调就是全速反转，
    几乎肯定是调用方写错了单位，所以这里直接报错而不是静默钳位。
    """
    v = int(value)
    if -100 <= v <= 100:
        return v
    if 1000 <= v <= 2000:
        return v
    raise ValueError(
        f"通道值 {v} 非法：只接受百分比 -100..100 或微秒 1000..2000。"
        f"（固件会把中间值钳到 1000us = 全速反转）"
    )


def firmware_decode_us(value: int) -> int:
    """复刻固件 ESC_ApplyToPWM：算出 STM32 实际会输出的脉宽。"""
    us = 1500 + value * 5 if -100 <= value <= 100 else value
    return max(1000, min(2000, us))


def firmware_would_accept(frame: bytes) -> bool:
    """复刻固件 spi_validate_frame：len==26 且 [0..2]==AA 01 10 且 crc8(前25)==[25]。"""
    if len(frame) != 26:
        return False
    if frame[0] != 0xAA or frame[1] != 0x01 or frame[2] != 16:
        return False
    return crc8(frame[:25]) == frame[25]


class ESC_SPI:
    FRAME_HEADER = 0xAA
    CMD = 0x01
    DATA_LEN = 16  # 8 * int16
    PADDING_LEN = 6
    TOTAL_FRAME_LEN = 26
    CHANNEL_COUNT = 8

    def __init__(self, bus=0, device=0, max_speed_hz=SPI_SPEED_HZ, mode=SPI_MODE,
                 bits_per_word=SPI_BITS_PER_WORD):
        if spidev is None:
            raise RuntimeError("spidev is required on Raspberry Pi to use SPI")
        if not (SPI_SPEED_MIN_HZ <= max_speed_hz <= SPI_SPEED_MAX_HZ):
            print(
                f"[警告] SPI 速率 {max_speed_hz} 不在固件验证过的 "
                f"{SPI_SPEED_MIN_HZ}–{SPI_SPEED_MAX_HZ} 区间，STM32 可能丢字节导致帧失步。",
                file=sys.stderr,
            )

        self.spi = spidev.SpiDev()
        try:
            self.spi.open(bus, device)
        except FileNotFoundError as exc:
            dev = f"/dev/spidev{bus}.{device}"
            spidev_list = []
            if os.path.isdir('/dev'):
                spidev_list = [n for n in os.listdir('/dev') if n.startswith('spidev')]
            raise RuntimeError(
                f"SPI device {dev} not found. Please enable SPI in raspi-config and verify the device exists. "
                f"Also confirm the correct bus/device and that the SPI kernel module is loaded. "
                f"Available SPI devices: {', '.join(sorted(spidev_list)) or 'none'}"
            ) from exc
        # ★ 必须逐项匹配 STM32 从机，不依赖 spidev 默认值
        self.spi.max_speed_hz = max_speed_hz
        self.spi.mode = mode
        self.spi.bits_per_word = bits_per_word
        self.spi.lsbfirst = False          # 固件 SPI_FIRSTBIT_MSB
        self.channels = [0] * self.CHANNEL_COUNT

    def describe(self) -> str:
        """回读驱动里的实际参数（Pi4 的 BCM2711 分频器与 Pi5 的 RP1 不同，实际速率会被取整）。"""
        return (f"mode={self.spi.mode} bits={self.spi.bits_per_word} "
                f"lsbfirst={self.spi.lsbfirst} speed={self.spi.max_speed_hz}Hz")

    def close(self):
        self.spi.close()

    def build_frame(self, channels):
        if len(channels) != self.CHANNEL_COUNT:
            raise ValueError(f"channels must be {self.CHANNEL_COUNT} values")

        frame = bytearray()
        frame.append(self.FRAME_HEADER)
        frame.append(self.CMD)
        frame.append(self.DATA_LEN)
        for v in channels:
            frame += struct.pack("<h", validate_channel_value(v))

        # 添加6个字节的填充
        frame.extend([0] * self.PADDING_LEN)

        frame.append(crc8(frame))
        # 验证帧长度
        if len(frame) != self.TOTAL_FRAME_LEN:
            raise RuntimeError(f"构建的帧长度 ({len(frame)}) 与预期的 ({self.TOTAL_FRAME_LEN}) 不符")
        return frame

    def send_frame(self, channels=None):
        """一次 xfer2 全双工传输 26 字节帧。
        发送 MOSI 控制帧，同时读取 MISO 状态帧并解析 STM32 反馈。
        """
        if channels is None:
            channels = self.channels
        frame = self.build_frame(channels)
        miso_raw = bytes(self.spi.xfer2(list(frame)))
        parse_stm32_miso(miso_raw)
        return frame

    @staticmethod
    def parse_frame(data: bytes):
        """★仅供离线自检：把一段字节按本协议解回 8 个通道值。
        链路状态请用 get_stm32_status() 读取 MISO 回读。
        """
        if not data:
            return None, "no data"
        if len(data) < ESC_SPI.TOTAL_FRAME_LEN:
            return None, f"len {len(data)} < {ESC_SPI.TOTAL_FRAME_LEN}"

        if data[0] != ESC_SPI.FRAME_HEADER:
            return None, f"bad header 0x{data[0]:02X}"
        if data[1] != ESC_SPI.CMD:
            return None, f"bad cmd 0x{data[1]:02X}"
        if data[2] != ESC_SPI.DATA_LEN:
            return None, f"bad len 0x{data[2]:02X}"

        frame_without_crc = data[:ESC_SPI.TOTAL_FRAME_LEN - 1]
        received_crc = data[ESC_SPI.TOTAL_FRAME_LEN - 1]
        calculated_crc = crc8(frame_without_crc)
        if calculated_crc != received_crc:
            return None, f"crc mismatch {calculated_crc:02X}!={received_crc:02X}"

        values = []
        for i in range(ESC_SPI.CHANNEL_COUNT):
            offset = 3 + i * 2
            values.append(int.from_bytes(data[offset : offset + 2], "little", signed=True))
        return values, "ok"

    @staticmethod
    def format_channel_value(value: int) -> str:
        if -100 <= value <= 100:
            us = 1500 + value * 5
            return f"{value:+4d}% / {us:4d}us"
        return f"{value:5d}us"

    def set_channel(self, index, value):
        if not 0 <= index < self.CHANNEL_COUNT:
            raise IndexError("channel index out of range")
        self.channels[index] = validate_channel_value(value)

    def set_all(self, values):
        if len(values) != self.CHANNEL_COUNT:
            raise ValueError(f"values must contain {self.CHANNEL_COUNT} channels")
        self.channels = [validate_channel_value(v) for v in values]

    def fill_center(self):
        """全通道停（百分比 0 → 固件输出 1500us 中位）。"""
        self.channels = [0] * self.CHANNEL_COUNT


# ───────────────────────── 离线自检（不碰硬件）─────────────────────────
def selftest() -> int:
    print("=== 离线帧自检（不发送、不需要硬件）===")
    cases = {
        "全停 0%": [0] * 8,
        "全前进 +100%": [100] * 8,
        "全后退 -100%": [-100] * 8,
        "百分比混合": [-100, -50, 0, 50, 100, 0, 0, 0],
        "直接微秒 1000..2000": [1000, 1100, 1200, 1300, 1400, 1600, 1800, 2000],
    }
    ok = True
    for name, channels in cases.items():
        frame = bytes(bytearray([0xAA, 0x01, 16])
                      + b"".join(struct.pack("<h", v) for v in channels)
                      + bytes(6))
        frame += bytes([crc8(frame)])
        accepted = firmware_would_accept(frame)
        decoded = [firmware_decode_us(v) for v in channels]
        if not accepted:
            ok = False
        print(f"\n[{'✓' if accepted else '✗'}] {name}")
        print(f"    帧({len(frame)}B): {' '.join(f'{b:02X}' for b in frame)}")
        print(f"    固件判定: {'有效' if accepted else '无效'}   CRC=0x{frame[25]:02X}")
        print(f"    STM32 会输出 us: {decoded}")

    print("\n--- 非法值应被拒绝 ---")
    for bad in (500, -500, 2500):
        try:
            validate_channel_value(bad)
            print(f"[✗] {bad} 竟然通过了校验")
            ok = False
        except ValueError:
            print(f"[✓] {bad} 已拒绝")

    print("\n=== 结果:", "全部通过 ✓" if ok else "有失败 ✗", "===")
    return 0 if ok else 1


# ───────────────────────── TUI ─────────────────────────
def draw_screen(stdscr, esc, selected, sent, hz):
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()

    def add_line(y, x, text):
        if y < 0 or y >= max_y:
            return
        text = text[:max_x - x]
        try:
            stdscr.addstr(y, x, text)
        except Exception:
            pass

    add_line(0, 0, "ESC SPI 控制面板 (q 退出, 1-8 选通道, ↑↓ 切通道, ←→ ±1%, c 全停)")
    add_line(1, 0, f"SPI: {esc.describe()} | {hz:.0f}Hz | 已发 {sent} 帧")
    st = get_stm32_status()
    add_line(2, 0, f"STM32: OK={st['stm32_frame_ok']} FAIL={st['stm32_frame_fail']} "
             f"INT={st['stm32_interrupt_count']} PWM={st['stm32_pwm_update']} "
             f"mode={st['stm32_mode_index']} miso={'OK' if st['miso_ok'] else 'N/A'}")
    add_line(4, 0, f"当前选中通道: {selected + 1}")
    add_line(5, 0, "发送通道值:")
    for idx, value in enumerate(esc.channels):
        mark = ">" if idx == selected else " "
        add_line(6 + idx, 0, f"{mark} 通道 {idx + 1}: {esc.format_channel_value(value)}")
    stdscr.refresh()


def tui_main(stdscr, speed_hz, hz):
    curses = __import__("curses")
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    esc = ESC_SPI(bus=0, device=0, max_speed_hz=speed_hz, mode=SPI_MODE)
    esc.fill_center()
    selected = 0
    sent = 0
    last_send = 0.0
    send_interval = 1.0 / hz

    def step(delta):
        cur = esc.channels[selected]
        # TUI 统一在百分比域操作，避开固件那段会被钳成全速反转的中间值
        if not -100 <= cur <= 100:
            cur = 0
        esc.set_channel(selected, max(-100, min(100, cur + delta)))

    try:
        while True:
            now = time.time()
            if now - last_send >= send_interval:
                # 与守护进程共用同一把总线锁，避免帧交错让 STM32 失步
                with spi_bus_lock():
                    esc.send_frame()
                sent += 1
                last_send = now
            draw_screen(stdscr, esc, selected, sent, hz)
            key = stdscr.getch()
            if key == -1:
                time.sleep(0.005)
                continue
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("c"), ord("C")):
                esc.fill_center()
            if key == curses.KEY_UP:
                selected = (selected - 1) % esc.CHANNEL_COUNT
            if key == curses.KEY_DOWN:
                selected = (selected + 1) % esc.CHANNEL_COUNT
            if key == curses.KEY_LEFT:
                step(-1)
            if key == curses.KEY_RIGHT:
                step(+1)
            if ord("1") <= key <= ord("8"):
                selected = key - ord("1")
    finally:
        # 固件没有失控保护，退出前务必把油门收回中位
        try:
            esc.fill_center()
            with spi_bus_lock():
                for _ in range(5):
                    esc.send_frame()
                    time.sleep(FRAME_GAP_SEC)
        except Exception:
            pass
        esc.close()


def main():
    ap = argparse.ArgumentParser(description="树莓派 → STM32 ESC SPI 单向下发")
    ap.add_argument("--selftest", action="store_true", help="离线帧自检（不发送、不需硬件）")
    ap.add_argument("--speed", type=int, default=SPI_SPEED_HZ,
                    help=f"SPI 速率 Hz（固件验证过 {SPI_SPEED_MIN_HZ}–{SPI_SPEED_MAX_HZ}）")
    ap.add_argument("--hz", type=float, default=50.0, help="发送频率")
    args = ap.parse_args()

    if args.selftest:
        sys.exit(selftest())

    try:
        import curses
    except ImportError:
        print("无法导入 curses，无法启动 TUI。请在支持 curses 的终端环境中运行。")
        return
    curses.wrapper(tui_main, args.speed, args.hz)


if __name__ == "__main__":
    main()
