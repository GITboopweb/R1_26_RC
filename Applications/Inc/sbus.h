#ifndef SBUS_H__
#define SBUS_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "usart.h" 


#ifdef __cplusplus
extern "C" {
#endif
#define MAX_SBUS_CHANNEL 16
#define SBUS_FRAME_SIZE  25

// 循环 DMA 接收缓冲区大小需大于单次轮询间隔的最大接收字节数；
// SBUS 波特率 100000 下每秒约 8333 字节，10ms 轮询间隔最多约 83 字节，因此取 128 字节留有充足余量。
#define SBUS_DMA_BUF_SIZE 128
#define SBUS_START_SYMBOL 0x0f

#define SBUS_INPUT_CHANNELS 16
#define SBUS_FLAGS_BYTE     23
#define SBUS_FAILSAFE_BIT   3
#define SBUS_FRAMELOST_BIT  2

#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))
typedef enum {
    SBUS_DECODE_STATE_DESYNC = 0xFFF,
    SBUS_DECODE_STATE_SBUS_START = 0x2FF,
    SBUS_DECODE_STATE_SBUS1_SYNC = 0x00,
    SBUS_DECODE_STATE_SBUS2_SYNC = 0x1FF,
    SBUS_DECODE_STATE_SBUS2_RX_VOLTAGE = 0x04,
    SBUS_DECODE_STATE_SBUS2_GPS = 0x14,
    SBUS_DECODE_STATE_SBUS2_DATA1 = 0x24,  // 预留拓展，当前版本未实现
    SBUS_DECODE_STATE_SBUS2_DATA2 = 0x34   // 预留拓展，当前版本未实现
} SBUS_DECODE_STATE;

typedef struct {
    uint16_t rc_count;
    uint16_t max_channels;
    bool sbus_failsafe;
    bool sbus_frame_drop;
    uint32_t sbus_frame_drops;
    uint32_t partial_frame_count;
    uint32_t last_rx_time;
    uint32_t last_frame_time;
    bool sbus_data_ready;
    SBUS_DECODE_STATE sbus_decode_state;
   
    uint8_t sbus_frame[SBUS_FRAME_SIZE + (SBUS_FRAME_SIZE / 2)];
    uint16_t sbus_val[MAX_SBUS_CHANNEL]; //(1000~2000)
} sbus_decoder_t;

typedef  struct
{
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t tsw1;
    int16_t ssw1;
    int16_t ssw2;
    int16_t tsw2;
    int16_t roll_1;
    int16_t roll_2;
    int16_t ch11;
    int16_t ch12;
    int16_t ch13;
    int16_t ch14;
    int16_t ch15;
    int16_t ch16;

    uint8_t frame_lost;
    uint8_t failsafe;
} __attribute__((packed)) sbus_info_t;
  
extern sbus_info_t channels;
#define SBUS_Init {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

void sbus_init(void);
void sbus_recv_poll(void);
sbus_info_t* sbus_get_datas(void);
void sbus_receive_callback(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}

#endif

#endif /* SBUS_H__ */
