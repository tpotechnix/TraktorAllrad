#ifndef _SBUS_H_
#define _SBUS_H_

#include <stdint.h>

#define SBUS_MIN        240
#define SBUS_MAX        1807

typedef enum {
    eFramePending = 0x00,
    eFrameComplete = 0x01,
    eFrameFaileSafe = 0x02,
    eFrameDropped = 0x04
} eSbusFrameState;

eSbusFrameState SbusProcess(uint8_t byte);
uint16_t SbusGetCh(int ch);
int Sbus2Rx(uint16_t sbus);
int SbusGetRx(int ch);

uint32_t SbusFrameCnt(void);

void SbusClearMinMax(void);
int SbusMaxDelta(void);

#endif // _SBUS_H_