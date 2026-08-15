#include "spi_slave.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "esc_spi.h"
#include <stdbool.h>
#include <string.h>

/* Simple SPI slave that receives bytes via interrupt and assembles frames.
   Assumes SPI2 is available (APB1). Pins: PB13 SCK, PB14 MISO, PB15 MOSI, PB12 NSS (optional)
*/

static SPI_HandleTypeDef hspi2;
static uint8_t spi_rx_buf[26];

volatile uint32_t spi_interrupt_count = 0;
volatile uint32_t spi_validate_ok_count = 0;
volatile uint32_t spi_validate_fail_count = 0;
volatile uint32_t spi_init_called = 0;
volatile uint8_t spi_nss_level = 0;
volatile uint8_t spi_use_software_nss = 0;

/* Debug snapshot of last received frame (for Keil Watch) */
volatile uint8_t spi_dbg_bytes[26] = {0};
volatile uint8_t spi_dbg_crc_calc = 0;
volatile uint8_t spi_dbg_hdr_ok = 0;
volatile uint32_t spi_exti_start_count = 0;
volatile uint32_t spi_exti_busy_skip_count = 0;
volatile uint8_t spi_mode_index = 0; /* 0..3: (CPOL,CPHA) = (0,0),(0,1),(1,0),(1,1) */
volatile uint8_t spi_pending_start = 0;
volatile uint32_t spi_poll_start_count = 0;
volatile uint32_t spi_poll_busy_skip_count = 0;
volatile uint32_t spi_poll_start_err_count = 0;
volatile uint32_t spi_pwm_update_count = 0;
volatile uint32_t spi_mode_switch_count = 0;
volatile uint8_t spi_pending_mode_switch = 0;
static void spi_apply_mode(uint8_t mode_index);

/* Framing diagnostics: search header inside last RX buffer and build an aligned view */
volatile uint8_t spi_dbg_hdr_pos = 0xFF;          /* 0..25 if found, else 0xFF */
volatile uint8_t spi_dbg_aligned_hdr_ok = 0;      /* aligned[0..2] == AA 01 10 */
volatile uint8_t spi_dbg_aligned_crc_ok = 0;      /* CRC of aligned[0..25] ok */
volatile uint8_t spi_dbg_aligned_crc_calc = 0;    /* calc CRC of aligned[0..24] */
volatile uint8_t spi_dbg_aligned_bytes[26] = {0}; /* aligned view (circular copy from rx) */

void SPI_Slave_Init(void)
{
    /* Strong sentinel: if this stays 0 in Watch, your firmware isn't running this function */
    spi_init_called = 0xA5A5A5A5;

    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure PB12-PB15 as AF push-pull for SPI2 slave.
       SCK/MOSI/NSS are inputs to the slave, MISO is output.
       On STM32F1, all must be GPIO_MODE_AF_PP to connect to the SPI peripheral. */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_SLAVE;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    /* Start with MODE0, we can sweep (0..3) at runtime via spi_mode_index if needed */
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT; /* software NSS — always selected, no need for PB12 wiring */
    /* Raspberry Pi Linux spidev is MSB-first for 8-bit transfers */
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;

    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        /* init error */
    }

    /* Ensure peripheral is enabled (some projects leave it disabled until first transfer) */
    __HAL_SPI_ENABLE(&hspi2);

    /* 读取 PB12 的当前电平，记录到 spi_nss_level（便于在 Watch 中观察） */
    spi_nss_level = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    spi_use_software_nss = 0; /* hardware NSS */

    /* IMPORTANT:
       Do NOT auto-switch to software NSS based on an idle-high read of PB12.
       NSS is expected to be high when idle; switching here would break hardware-CS operation.
       If you want to test software NSS, do it explicitly by configuration.
    */

    debug_spi_frame_len = 0;
    debug_spi_frame_count = 0;
    debug_spi_last_byte = 0;
    debug_spi_frame_ready = 0;
    debug_spi_error = 0;

    /* Enable SPI IRQ to handle NSS transitions if needed */
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);

    /* Pre-arm first receive BEFORE CS goes low to avoid missing the header. */
    memset((void*)spi_rx_buf, 0, sizeof(spi_rx_buf));
    HAL_SPI_Receive_IT(&hspi2, spi_rx_buf, sizeof(spi_rx_buf));

    /* 配置 PA0 为推挽输出并在初始化完成时翻转一次，便于用示波器/LED 观察初始化是否发生 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = GPIO_PIN_0;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    /* Toggle PA0 to indicate init happened */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

    /* Mark init done (overwrites sentinel) */
    spi_init_called = 1;
}

void SPI_Slave_StartReceive(void)
{
    debug_spi_frame_len = 0;
    debug_spi_frame_ready = 0;
    debug_spi_error = 0;
    /* When using EXTI gating, the transfer is started on CS falling edge. */
}

/*
 * Call frequently from main loop (non-ISR context).
 * Starts a fresh 26-byte full-duplex transfer after CS falling edge (EXTI sets spi_pending_start).
 */
void SPI_Slave_Poll(void)
{
    if (spi_pending_mode_switch)
    {
        if (hspi2.State == HAL_SPI_STATE_READY)
        {
            spi_pending_mode_switch = 0;
            spi_apply_mode(spi_mode_index);
            spi_mode_switch_count++;
        }
    }

    /* 仅在接收出错后兜底重启；正常由 ISR 自行 re-arm，避免主循环打乱帧对齐 */
    if (hspi2.State == HAL_SPI_STATE_READY && debug_spi_error) {
        spi_poll_start_count++;
        memset((void*)spi_rx_buf, 0, sizeof(spi_rx_buf));
        if (HAL_SPI_Receive_IT(&hspi2, spi_rx_buf, sizeof(spi_rx_buf)) != HAL_OK) {
            spi_poll_start_err_count++;
        } else {
            debug_spi_error = 0;
        }
        return;
    }

    if (hspi2.State != HAL_SPI_STATE_READY)
    {
        spi_poll_busy_skip_count++;
        return;
    }
}

void SPI_Slave_IRQHandler(void)
{
    /* Placeholder: users can call HAL_SPI_IRQHandler(&hspi2) in global IRQ handler */
    HAL_SPI_IRQHandler(&hspi2);
}

static uint8_t crc8(const volatile uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

static void spi_apply_mode(uint8_t mode_index)
{
    uint8_t idx = mode_index & 0x03;
    if (idx == 0) {
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else if (idx == 1) {
        hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
        hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    } else if (idx == 2) {
        hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    } else {
        hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
        hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    }

    HAL_SPI_DeInit(&hspi2);
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        return;
    }
    __HAL_SPI_ENABLE(&hspi2);

    memset((void*)spi_rx_buf, 0, sizeof(spi_rx_buf));
    HAL_SPI_Receive_IT(&hspi2, spi_rx_buf, sizeof(spi_rx_buf));
}

static bool spi_validate_frame(const uint8_t *buf, uint16_t len)
{
    if (len != 26) return false;
    if (buf[0] != 0xAA || buf[1] != 0x01 || buf[2] != 16) return false;
    return crc8(buf, 25) == buf[25];
}

static uint8_t find_header_pos(const uint8_t *buf, uint16_t len)
{
    /* Look for AA 01 10 sequence anywhere inside the received window */
    if (len < 3) return 0xFF;
    for (uint16_t i = 0; i + 2 < len; ++i)
    {
        if (buf[i] == 0xAA && buf[i + 1] == 0x01 && buf[i + 2] == 16)
            return (uint8_t)i;
    }
    return 0xFF;
}

static void build_aligned_view(const uint8_t *rx)
{
    spi_dbg_hdr_pos = find_header_pos(rx, 26);
    spi_dbg_aligned_hdr_ok = 0;
    spi_dbg_aligned_crc_ok = 0;
    spi_dbg_aligned_crc_calc = 0;
    memset((void*)spi_dbg_aligned_bytes, 0, sizeof(spi_dbg_aligned_bytes));

    if (spi_dbg_hdr_pos == 0xFF)
        return;

    /* Circularly rotate the 26-byte window so that header starts at index 0 */
    for (uint8_t j = 0; j < 26; ++j)
        spi_dbg_aligned_bytes[j] = rx[(uint8_t)((spi_dbg_hdr_pos + j) % 26)];

    if (spi_dbg_aligned_bytes[0] == 0xAA && spi_dbg_aligned_bytes[1] == 0x01 && spi_dbg_aligned_bytes[2] == 16)
        spi_dbg_aligned_hdr_ok = 1;

    spi_dbg_aligned_crc_calc = crc8(spi_dbg_aligned_bytes, 25);
    if (spi_dbg_aligned_crc_calc == spi_dbg_aligned_bytes[25])
        spi_dbg_aligned_crc_ok = 1;
}

/* This callback is called by HAL when RX complete */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) {
        spi_interrupt_count++; // 中断计数

        /* Snapshot what we actually received to quickly diagnose: header mismatch vs CRC mismatch */
        memcpy((void *)spi_dbg_bytes, spi_rx_buf, sizeof(spi_dbg_bytes));
        spi_dbg_hdr_ok = (spi_rx_buf[0] == 0xAA && spi_rx_buf[1] == 0x01 && spi_rx_buf[2] == 16) ? 1u : 0u;
    spi_dbg_crc_calc = crc8(spi_rx_buf, 25);

    /* Build aligned view to determine framing vs sampling issues */
    build_aligned_view(spi_rx_buf);

        if (spi_validate_frame(spi_rx_buf, sizeof(spi_rx_buf))) {
            debug_spi_frame_ready = 1;
            debug_spi_error = 0;
            spi_validate_ok_count++;
            ESC_UpdateFromBuffer(spi_rx_buf, sizeof(spi_rx_buf));
            ESC_ApplyToPWM();
            spi_pwm_update_count++;
            debug_spi_frame_count++;
        } else if (spi_dbg_aligned_crc_ok) {
            debug_spi_frame_ready = 1;
            debug_spi_error = 0;
            spi_validate_ok_count++;
            ESC_UpdateFromBuffer((uint8_t *)spi_dbg_aligned_bytes, sizeof(spi_dbg_aligned_bytes));
            ESC_ApplyToPWM();
            spi_pwm_update_count++;
            debug_spi_frame_count++;
        } else if (spi_dbg_hdr_pos != 0xFF) {
            /* 次优：在窗口内找到帧头但 CRC 未过，仍尝试按对齐位置解析油门 */
            uint8_t tmp[26];
            for (uint8_t j = 0; j < 26; ++j)
                tmp[j] = spi_rx_buf[(uint8_t)((spi_dbg_hdr_pos + j) % 26)];
            if (tmp[0] == 0xAA && tmp[1] == 0x01 && tmp[2] == 16) {
                ESC_UpdateFromBuffer(tmp, sizeof(tmp));
                ESC_ApplyToPWM();
                spi_pwm_update_count++;
            }
            debug_spi_frame_ready = 0;
            debug_spi_error = 1;
            spi_validate_fail_count++;
        } else {
            debug_spi_frame_ready = 0;
            debug_spi_error = 1;
            spi_validate_fail_count++;
        }

        /* Always arm next receive immediately to capture the next frame header. */
        HAL_SPI_Receive_IT(&hspi2, spi_rx_buf, sizeof(spi_rx_buf));
    }
}

/* RX-only mode completion callback */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_TxRxCpltCallback(hspi);
}

/* IRQ handler tied to vector table - forward to HAL */
void SPI2_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi2);
}

/* EXTI callback when PB12 (NSS) changes: start a fresh SPI IT transfer on falling edge */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
    /* EXTI not used in pre-armed mode; left empty intentionally. */
}

/* EXTI vector */
void EXTI15_10_IRQHandler(void)
{
    /* EXTI not used in pre-armed mode; leave handler empty. */
}
