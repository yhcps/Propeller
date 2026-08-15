#!/usr/bin/env bash
# stlink_pwm.sh — 通过 ST-Link (OpenOCD) 直接读写 8 路推进器 PWM
#
# 用法:
#   ./stlink_pwm.sh read          # 读取全部 8 路 PWM 当前值
#   ./stlink_pwm.sh set <ch> <us> # 设置通道 <ch>(0-7) 为 <us>us (1000-2000)
#   ./stlink_pwm.sh all <us>      # 设置全部 8 路为 <us>us
#   ./stlink_pwm.sh ramp <ch>     # 通道 <ch> 从 1000→2000→1000 扫一遍
#   ./stlink_pwm.sh stop          # 全部回到中位 1500us
#   ./stlink_pwm.sh test <ch>     # 通道 <ch> 输出 1500→1000→1500→2000→1500
#
# 依赖: openocd (树莓派已安装), sudo 权限

set -euo pipefail

# ── 配置 ──
OPENOCD="/usr/share/openocd/scripts"
INTERFACE="${OPENOCD}/interface/stlink.cfg"
TARGET="${OPENOCD}/target/stm32f1x.cfg"
SPEED=100

# TIM1/TIM4 CCR 寄存器地址
# TIM1 base 0x40012C00, TIM4 base 0x40000800, CCR1..4 offset 0x34,0x38,0x3C,0x40
declare -A CCR_ADDR=(
  [0]="0x40012C34"   # TIM1_CH1 → PA8
  [1]="0x40012C38"   # TIM1_CH2 → PA9
  [2]="0x40012C3C"   # TIM1_CH3 → PA10
  [3]="0x40012C40"   # TIM1_CH4 → PA11
  [4]="0x40000834"   # TIM4_CH1 → PB6
  [5]="0x40000838"   # TIM4_CH2 → PB7
  [6]="0x4000083C"   # TIM4_CH3 → PB8
  [7]="0x40000840"   # TIM4_CH4 → PB9
)

declare -A CH_GPIO=(
  [0]="PA8"
  [1]="PA9"
  [2]="PA10"
  [3]="PA11"
  [4]="PB6"
  [5]="PB7"
  [6]="PB8"
  [7]="PB9"
)

SUDO_PASS="200655"

# ── 工具函数 ──
openocd_cmd() {
  local cmd="$1"
  echo "$SUDO_PASS" | sudo -S openocd \
    -f "$INTERFACE" \
    -f "$TARGET" \
    -c "adapter speed $SPEED" \
    -c "transport select hla_swd" \
    -c "init" \
    -c "$cmd" \
    -c "shutdown" 2>&1
}

# 读取单个 CCR 寄存器
read_ccr() {
  local ch="$1"
  local addr="${CCR_ADDR[$ch]}"
  local val
  val=$(openocd_cmd "mdw $addr 1" | grep -oP '0x[0-9a-fA-F]+: \K[0-9a-fA-F]+')
  echo "$((16#$val))"
}

# 写入单个 CCR 寄存器
write_ccr() {
  local ch="$1"
  local us="$2"
  local addr="${CCR_ADDR[$ch]}"
  # 验证范围
  if [ "$us" -lt 900 ] || [ "$us" -gt 2100 ]; then
    echo "错误: 脉冲宽度 $us us 超出范围 (900-2100)" >&2
    return 1
  fi
  openocd_cmd "mww $addr $us" > /dev/null
  echo "通道 $ch (${CH_GPIO[$ch]}): → ${us}us"
}

# ── 命令处理 ──
cmd="${1:-help}"

case "$cmd" in
  read)
    echo "===== 8路推进器 PWM 当前值 ====="
    echo "通道 | GPIO | 脉冲 (us) | 百分比"
    echo "-----|------|-----------|-------"
    for ch in {0..7}; do
      val=$(read_ccr "$ch")
      pct=$(awk "BEGIN {printf \"%.1f\", ($val-1500)/5.0}")
      printf "  %d   | %-4s | %5d    | %6s%%\n" "$ch" "${CH_GPIO[$ch]}" "$val" "$pct"
    done
    ;;

  set)
    ch="${2:?用法: $0 set <通道0-7> <脉宽us>}"
    us="${3:?用法: $0 set <通道0-7> <脉宽us>}"
    if [ "$ch" -lt 0 ] || [ "$ch" -gt 7 ]; then
      echo "错误: 通道 $ch 无效 (0-7)" >&2
      exit 1
    fi
    write_ccr "$ch" "$us"
    ;;

  all)
    us="${2:?用法: $0 all <脉宽us>}"
    echo "设置全部 8 路为 ${us}us..."
    for ch in {0..7}; do
      write_ccr "$ch" "$us"
    done
    ;;

  stop)
    echo "全部 8 路回到中位 1500us..."
    for ch in {0..7}; do
      write_ccr "$ch" 1500
    done
    ;;

  ramp)
    ch="${2:?用法: $0 ramp <通道0-7>}"
    if [ "$ch" -lt 0 ] || [ "$ch" -gt 7 ]; then
      echo "错误: 通道 $ch 无效 (0-7)" >&2
      exit 1
    fi
    echo "通道 $ch (${CH_GPIO[$ch]}) 扫频: 1000 → 2000 → 1000 us"
    for us in $(seq 1000 20 2000) $(seq 1980 -20 1000); do
      write_ccr "$ch" "$us"
      sleep 0.05
    done
    write_ccr "$ch" 1500
    echo "完成，回到中位"
    ;;

  test)
    ch="${2:?用法: $0 test <通道0-7>}"
    if [ "$ch" -lt 0 ] || [ "$ch" -gt 7 ]; then
      echo "错误: 通道 $ch 无效 (0-7)" >&2
      exit 1
    fi
    echo "通道 $ch (${CH_GPIO[$ch]}) 测试: 1500→1000→1500→2000→1500 us"
    for us in 1500 1000 1500 2000 1500; do
      write_ccr "$ch" "$us"
      sleep 2
    done
    echo "完成"
    ;;

  help|--help|-h)
    echo "用法: $0 <命令> [参数]"
    echo ""
    echo "命令:"
    echo "  read             读取全部 8 路 PWM 当前值"
    echo "  set <ch> <us>    设置通道 <ch>(0-7) 为 <us>us (1000-2000)"
    echo "  all <us>         设置全部 8 路为 <us>us"
    echo "  ramp <ch>        通道 <ch> 从 1000→2000→1000 扫一遍"
    echo "  stop             全部回到中位 1500us"
    echo "  test <ch>        通道 <ch> 输出 1500→1000→1500→2000→1500"
    echo ""
    echo "通道映射:"
    echo "  ch0: TIM1_CH1 PA8    ch4: TIM4_CH1 PB6"
    echo "  ch1: TIM1_CH2 PA9    ch5: TIM4_CH2 PB7"
    echo "  ch2: TIM1_CH3 PA10   ch6: TIM4_CH3 PB8"
    echo "  ch3: TIM1_CH4 PA11   ch7: TIM4_CH4 PB9"
    echo ""
    echo "PWM: 50Hz, 1MHz tick, 1000us=停止/反转, 1500us=中位, 2000us=全速"
    ;;

  *)
    echo "未知命令: $cmd" >&2
    echo "运行 '$0 help' 查看用法" >&2
    exit 1
    ;;
esac