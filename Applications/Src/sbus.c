#include "sbus.h"
#include "main.h" 


sbus_decoder_t      s_decoder;              // SBUS 解析器实例（原 main.c 的 my_sbus）
static UART_HandleTypeDef* s_sbus_huart = NULL;    // 绑定的串口（本项目为 huart1）
uint8_t  s_sbus_dma_buf[SBUS_DMA_BUF_SIZE]; // DMA 循环接收缓冲
sbus_datas datas;
static volatile uint16_t s_sbus_rd_pos = 0;        // 已处理到的读指针（追 DMA 写指针）

sbus_info_t channels = SBUS_Init;

struct sbus_bit_pick {
    uint8_t byte;
    uint8_t rshift;
    uint8_t mask;
    uint8_t lshift;
};

static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] =
	{
    { { 0, 0, 0xff, 0 }, { 1, 0, 0x07, 8 }, { 0, 0, 0x00, 0 } },
    { { 1, 3, 0x1f, 0 }, { 2, 0, 0x3f, 5 }, { 0, 0, 0x00, 0 } },
    { { 2, 6, 0x03, 0 }, { 3, 0, 0xff, 2 }, { 4, 0, 0x01, 10 } },
    { { 4, 1, 0x7f, 0 }, { 5, 0, 0x0f, 7 }, { 0, 0, 0x00, 0 } },
    { { 5, 4, 0x0f, 0 }, { 6, 0, 0x7f, 4 }, { 0, 0, 0x00, 0 } },
    { { 6, 7, 0x01, 0 }, { 7, 0, 0xff, 1 }, { 8, 0, 0x03, 9 } },
    { { 8, 2, 0x3f, 0 }, { 9, 0, 0x1f, 6 }, { 0, 0, 0x00, 0 } },
    { { 9, 5, 0x07, 0 }, { 10, 0, 0xff, 3 }, { 0, 0, 0x00, 0 } },
    { { 11, 0, 0xff, 0 }, { 12, 0, 0x07, 8 }, { 0, 0, 0x00, 0 } },
    { { 12, 3, 0x1f, 0 }, { 13, 0, 0x3f, 5 }, { 0, 0, 0x00, 0 } },
    { { 13, 6, 0x03, 0 }, { 14, 0, 0xff, 2 }, { 15, 0, 0x01, 10 } },
    { { 15, 1, 0x7f, 0 }, { 16, 0, 0x0f, 7 }, { 0, 0, 0x00, 0 } },
    { { 16, 4, 0x0f, 0 }, { 17, 0, 0x7f, 4 }, { 0, 0, 0x00, 0 } },
    { { 17, 7, 0x01, 0 }, { 18, 0, 0xff, 1 }, { 19, 0, 0x03, 9 } },
    { { 19, 2, 0x3f, 0 }, { 20, 0, 0x1f, 6 }, { 0, 0, 0x00, 0 } },
    { { 20, 5, 0x07, 0 }, { 21, 0, 0xff, 3 }, { 0, 0, 0x00, 0 } }
};

static bool sbus_decode(sbus_decoder_t* decoder, uint32_t frame_time)
{
    if ((decoder->sbus_frame[0] != SBUS_START_SYMBOL)) {
        decoder->sbus_frame_drops++;
        decoder->sbus_decode_state = SBUS_DECODE_STATE_DESYNC;
        return false;
    }

    switch (decoder->sbus_frame[24]) {
    case 0x00:
        decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS1_SYNC;
        break;
    case 0x04:
        decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS2_RX_VOLTAGE;
        break;
    case 0x14:
        decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS2_GPS;
        break;
    case 0x24:
    case 0x34:
        decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS2_SYNC;
        break;
    default:
        decoder->sbus_decode_state = SBUS_DECODE_STATE_DESYNC;
        return false;
    }

    decoder->last_frame_time = frame_time;
    unsigned chancount = (decoder->max_channels > SBUS_INPUT_CHANNELS) ? SBUS_INPUT_CHANNELS : decoder->max_channels;

    for (unsigned channel = 0; channel < chancount; channel++) {
        unsigned value = 0;
        for (unsigned pick = 0; pick < 3; pick++) {
            const struct sbus_bit_pick* decode = &sbus_decoder[channel][pick];
            if (decode->mask != 0) {
                unsigned piece = decoder->sbus_frame[1 + decode->byte];
                piece >>= decode->rshift;
                piece &= decode->mask;
                piece <<= decode->lshift;
                value |= piece;
            }
        }
        decoder->sbus_val[channel] = (uint16_t)(value * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
    }

    decoder->rc_count = chancount;

    if (decoder->sbus_frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) {
        decoder->sbus_failsafe = true;
        decoder->sbus_frame_drop = true;
    } else if (decoder->sbus_frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) {
        decoder->sbus_failsafe = false;
        decoder->sbus_frame_drop = true;
    } else {
        decoder->sbus_failsafe = false;
        decoder->sbus_frame_drop = false;
    }

    return true;
}

static bool sbus_parse(sbus_decoder_t* decoder, uint8_t* frame, unsigned len)
{
    unsigned i;
    decoder->last_rx_time = HAL_GetTick();
    bool decode_ret = false;

    for (i = 0; i < len; i++) {
        if (decoder->partial_frame_count == sizeof(decoder->sbus_frame) / sizeof(decoder->sbus_frame[0])) {
            decoder->partial_frame_count = 0;
            decoder->sbus_decode_state = SBUS_DECODE_STATE_DESYNC;
        }

        if (decoder->partial_frame_count == SBUS_FRAME_SIZE) {
            decoder->partial_frame_count = 0;
            decoder->sbus_decode_state = SBUS_DECODE_STATE_DESYNC;
        }

        switch (decoder->sbus_decode_state) {
        case SBUS_DECODE_STATE_DESYNC:
            if (frame[i] == SBUS_START_SYMBOL) {
                decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS_START;
                decoder->partial_frame_count = 0;
                decoder->sbus_frame[decoder->partial_frame_count++] = frame[i];
            }
            break;

        case SBUS_DECODE_STATE_SBUS_START:
        case SBUS_DECODE_STATE_SBUS1_SYNC:
        case SBUS_DECODE_STATE_SBUS2_SYNC: {
            decoder->sbus_frame[decoder->partial_frame_count++] = frame[i];
            if (decoder->partial_frame_count < SBUS_FRAME_SIZE) {
                break;
            }

            decode_ret = sbus_decode(decoder, decoder->last_rx_time);
            unsigned start_index = 0;

            if (!decode_ret && decoder->sbus_decode_state == SBUS_DECODE_STATE_DESYNC) {
                for (unsigned j = 1; j < decoder->partial_frame_count; j++) {
                    if (decoder->sbus_frame[j] == SBUS_START_SYMBOL) {
                        start_index = j;
                        break;
                    }
                }
                if (start_index != 0) {
                    for (unsigned j = 0; j < decoder->partial_frame_count - start_index; j++) {
                        decoder->sbus_frame[j] = decoder->sbus_frame[j + start_index];
                    }
                    decoder->partial_frame_count -= start_index;
                    decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS_START;
                }
            }
            if (start_index == 0) {
                decoder->partial_frame_count = 0;
            }
        } break;

        case SBUS_DECODE_STATE_SBUS2_RX_VOLTAGE:
        case SBUS_DECODE_STATE_SBUS2_GPS:
            decoder->sbus_frame[decoder->partial_frame_count++] = frame[i];
            if (decoder->partial_frame_count == 1 && decoder->sbus_frame[0] == SBUS_START_SYMBOL) {
                decoder->sbus_decode_state = SBUS_DECODE_STATE_SBUS2_SYNC;
            }
            break;

        default:
            decode_ret = false;
        }
    }

    decoder->sbus_data_ready = decode_ret && !decoder->sbus_failsafe && !decoder->sbus_frame_drop;
    return decode_ret;
}

static int sbus_decoder_init(sbus_decoder_t* decoder)
{
    if (decoder == NULL) {
        return -1;
    }
    memset(decoder, 0, sizeof(sbus_decoder_t));
    decoder->sbus_decode_state = SBUS_DECODE_STATE_DESYNC;
    decoder->max_channels = MAX_SBUS_CHANNEL;
    return 0;
}

// 绑定串口并启动循环 DMA 接收（内部使用）。
static void sbus_recv_start(UART_HandleTypeDef* huart)
{
    s_sbus_huart = huart;

    s_sbus_rd_pos = 0;
    HAL_UART_Receive_DMA(huart, s_sbus_dma_buf, SBUS_DMA_BUF_SIZE);

    // 关掉 UART 的校验(PE)和错误(ERR: FE/NE/ORE)中断：SBUS 偶发坏字节是常态，关掉后
    // 坏字节静默进缓冲，由 sbus_parse() 靠帧头/状态机过滤，DMA 永不被打断。
    __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

    // 使能 IDLE 空闲中断
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

// 初始化函数
void sbus_init(void)
{
    sbus_decoder_init(&s_decoder);
    sbus_recv_start(&huart3);
}

// 轮询 DMA 写指针，把新收到的字节喂给 sbus_parse()，解析结果存进内部 s_decoder。
void sbus_recv_poll(void)
{
    if (s_sbus_huart == NULL) {
        return;
    }
//	HAL_UART_DMAStop(&huart3);
    
		// DMA_GET_COUNTER 是“剩余未传输字节数”，用缓冲大小减去它即得当前写指针位置（含回绕）。
    uint16_t dma_pos = SBUS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(s_sbus_huart->hdmarx);
    while (s_sbus_rd_pos != dma_pos)
    {
        sbus_parse(&s_decoder, &s_sbus_dma_buf[s_sbus_rd_pos], 1);
        if (++s_sbus_rd_pos >= SBUS_DMA_BUF_SIZE)
            s_sbus_rd_pos = 0;
    }
    sbus_get_datas();
		
//  HAL_UART_Receive_DMA(&huart3, s_sbus_dma_buf, SBUS_DMA_BUF_SIZE);

}

// 获取 SBUS 解析器，供外部读取解析后的通道数据（s_decoder.sbus_val[] 等）。
sbus_info_t* sbus_get_datas(void)
{
    
  channels.ch1    = s_decoder.sbus_val[0];
  channels.ch2    = s_decoder.sbus_val[1];
  channels.ch3    = s_decoder.sbus_val[2];
  channels.ch4    = s_decoder.sbus_val[3];
  channels.tsw1   = s_decoder.sbus_val[4];
  channels.ssw1   = s_decoder.sbus_val[5];
  channels.ssw2   = s_decoder.sbus_val[6];
  channels.tsw2   = s_decoder.sbus_val[7];
  channels.roll_1 = s_decoder.sbus_val[8];
  channels.roll_2 = s_decoder.sbus_val[9];
  channels.ch11   = s_decoder.sbus_val[10];
  channels.ch12   = s_decoder.sbus_val[11];
  channels.ch13   = s_decoder.sbus_val[12];
  channels.ch14   = s_decoder.sbus_val[13];
  channels.ch15   = s_decoder.sbus_val[14];
  channels.ch16   = s_decoder.sbus_val[15];

  channels.frame_lost = s_decoder.sbus_frame_drop;
  channels.failsafe   = s_decoder.sbus_failsafe;
    return &channels;
}
void sbus_receive_callback(UART_HandleTypeDef* huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
       
			sbus_recv_poll();
    }
}

