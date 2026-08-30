/* CH559 USB gamepad -> UART5 receiver, based on the differential project. */
#include "ps2_usart.h"

#include "stm32f1xx_hal.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

static uint8_t s_rx_byte;
static uint8_t s_work_buf[RX_BUF_SIZE];
static uint16_t s_work_count;

uint8_t uart5_rx_buf[RX_BUF_SIZE];
volatile uint8_t uart5_rx_finish = 0U;
volatile uint32_t uart5_rx_frame_tick = 0U;

PS2_HandleTypeDef ps2 = {0};
volatile uint8_t data_flag = 0U;
volatile uint8_t remote_stopped_flag = 1U;
volatile uint32_t ps2_last_frame_tick = 0U;

static int map_joystick(unsigned int raw)
{
    return (int)raw - 128;
}

void usart_rx_init(void)
{
    s_work_count = 0U;
    uart5_rx_finish = 0U;
    uart5_rx_frame_tick = 0U;
    data_flag = 0U;
    remote_stopped_flag = 1U;
    (void)HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        if ((s_rx_byte == (uint8_t)'\n') || (s_work_count >= (RX_BUF_SIZE - 2U)))
        {
            s_work_buf[s_work_count] = '\0';

            /* Keep one complete line stable while the control task parses it. */
            if (uart5_rx_finish == 0U)
            {
                memcpy(uart5_rx_buf, s_work_buf, (size_t)(s_work_count + 1U));
                uart5_rx_frame_tick = HAL_GetTick();
                uart5_rx_finish = 1U;
            }
            s_work_count = 0U;
        }
        else if (s_rx_byte != (uint8_t)'\r')
        {
            s_work_buf[s_work_count++] = s_rx_byte;
        }

        (void)HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        s_work_count = 0U;
        __HAL_UART_CLEAR_OREFLAG(&huart5);
        (void)HAL_UART_Receive_IT(&huart5, &s_rx_byte, 1U);
    }
}

void ps2_parse_data(uint8_t *str)
{
    unsigned int x1, x2, x3, x4, x5, x6, x7, x8;
    int parsed;

    parsed = sscanf(
        (char *)str,
        "HUB0_Joystick data: x%02X x%02X x%02X x%02X x%02X x%02X x%02X x%02X",
        &x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);

    if ((parsed != 8) || (x1 != 0x01U))
    {
        return;
    }

    ps2.frame_id = (uint8_t)x1;
    ps2.lx = (int16_t)map_joystick(x4);
    ps2.ly = (int16_t)map_joystick(x5);
    ps2.left_key = 0U;
    ps2.dpad_buttons = 0U;
    ps2.right_key = 0U;
    ps2.shoulder_buttons = 0U;
    ps2.select = 0U;

    switch (x5)
    {
        case KEY_UP:
            ps2.left_key = 1U;
            ps2.dpad_buttons |= PS2_DPAD_UP;
            break;
        case KEY_DOWN:
            ps2.left_key = 2U;
            ps2.dpad_buttons |= PS2_DPAD_DOWN;
            break;
        default: break;
    }
    switch (x4)
    {
        case KEY_LEFT:
            ps2.left_key = 3U;
            ps2.dpad_buttons |= PS2_DPAD_LEFT;
            break;
        case KEY_RIGHT:
            ps2.left_key = 4U;
            ps2.dpad_buttons |= PS2_DPAD_RIGHT;
            break;
        default: break;
    }

    /* Exact right-control mapping retained from the reference code. */
    switch (x6)
    {
        case KEY_Y: ps2.right_key = 1U; break;
        case KEY_A: ps2.right_key = 2U; break;
        case KEY_X: ps2.right_key = 3U; break;
        case KEY_B: ps2.right_key = 4U; break;
        default: break;
    }

    /* x7 codes are independent bits. Keep combinations such as SELECT+L1. */
    ps2.shoulder_buttons = (uint8_t)(x7 & PS2_SHOULDER_MASK);
    ps2.select = ((x7 & KEY_SELECTION) != 0U) ? 1U : 0U;

    /* Every valid report, including a centered report, is new control data. */
    data_flag = 1U;
    remote_stopped_flag = 0U;
    /* Preserve wire-reception time; a delayed task must not renew an old frame. */
    ps2_last_frame_tick = uart5_rx_frame_tick;
}

void ps2_update_timeout(uint32_t now_ms, uint32_t timeout_ms)
{
    if ((uint32_t)(now_ms - ps2_last_frame_tick) > timeout_ms)
    {
        remote_stopped_flag = 1U;
        data_flag = 0U;
        ps2.lx = 0;
        ps2.ly = 0;
        ps2.left_key = 0U;
        ps2.dpad_buttons = 0U;
        ps2.right_key = 0U;
        ps2.shoulder_buttons = 0U;
        ps2.select = 0U;
    }
}
