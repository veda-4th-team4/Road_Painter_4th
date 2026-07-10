#ifndef __PACKET_PARSER_H__
#define __PACKET_PARSER_H__

#include "main.h"

#define RX_RING_BUF_SIZE 256

typedef enum {
    STATE_STX, STATE_LEN, STATE_CMD, STATE_PAYLOAD, STATE_CRC, STATE_ETX
} ParserState_t;

// [명령셋 구조체 정의 일치화]
#pragma pack(push, 1)
typedef struct {
    int16_t left_sps;
    int16_t right_sps;
} Msg_SetSpeed_t; // CMD 0x01 (4 Bytes)

typedef struct {
    uint8_t nozzle_on;
} Msg_ControlNozzle_t; // CMD 0x02 (1 Byte)

typedef struct {
    uint8_t fault_reason;
} Msg_EStop_t; // CMD 0x03 (1 Byte)
#pragma pack(pop)

// 메인 루프나 타이머 인터럽트에서 참조할 글로벌 제어 변수들
extern volatile int16_t global_left_sps;
extern volatile int16_t global_right_sps;
extern volatile uint8_t global_nozzle_on;
extern volatile uint8_t global_estop_flag;

void Packet_Parser_Init(UART_HandleTypeDef *huart);
void Packet_Parser_Push_Byte(uint8_t byte);
void Packet_Parser_Process(void);

#endif /* __PACKET_PARSER_H__ */
