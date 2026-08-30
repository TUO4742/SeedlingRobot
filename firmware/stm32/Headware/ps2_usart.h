#ifndef __PS2_USART_H
#define __PS2_USART_H

#include <stdint.h>

#define RX_BUF_SIZE 128U

/* Keep the CH559 report mapping used by the differential-chassis project. */
#define KEY_MID       0x0FU
#define KEY_UP        0x00U
#define KEY_DOWN      0xFFU
#define KEY_LEFT      0x00U
#define KEY_RIGHT     0xFFU

/* x6: face/right-control codes from the board's CH559 firmware. */
#define KEY_Y         0x1FU
#define KEY_A         0x4FU
#define KEY_X         0x8FU
#define KEY_B         0x2FU

/* x7: shoulder/select codes. */
#define KEY_L1        0x01U
#define KEY_L2        0x04U
#define KEY_R1        0x02U
#define KEY_R2        0x08U
#define KEY_SELECTION 0x10U

#define PS2_SHOULDER_MASK 0x0FU
#define PS2_DPAD_UP       0x01U
#define PS2_DPAD_DOWN     0x02U
#define PS2_DPAD_LEFT     0x04U
#define PS2_DPAD_RIGHT    0x08U

typedef struct
{
    uint8_t frame_id;     /* 0x01 is a valid controller report. */
    int16_t lx;           /* Left stick X, mapped to -128..127. */
    int16_t ly;           /* Left stick Y, mapped to -128..127. */
    uint8_t left_key;     /* 0 none, 1 up, 2 down, 3 left, 4 right. */
    uint8_t dpad_buttons; /* Bit mask; preserves diagonal/conflicting input. */
    uint8_t right_key;    /* 0 none, 1 Y/up, 2 A/down, 3 X/left, 4 B/right. */
    uint8_t shoulder_buttons; /* x7 bit mask: L1/R1/L2/R2 = bits 0..3. */
    uint8_t select : 1;
} PS2_HandleTypeDef;

extern volatile uint8_t uart5_rx_finish;
extern uint8_t uart5_rx_buf[RX_BUF_SIZE];
extern volatile uint32_t uart5_rx_frame_tick;
extern PS2_HandleTypeDef ps2;
extern volatile uint8_t data_flag;
extern volatile uint8_t remote_stopped_flag;
extern volatile uint32_t ps2_last_frame_tick;

void usart_rx_init(void);
void ps2_parse_data(uint8_t *str);
void ps2_update_timeout(uint32_t now_ms, uint32_t timeout_ms);

#endif
