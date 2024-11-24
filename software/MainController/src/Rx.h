#ifndef _RX_H_
#define _RX_H_

#include "stdint.h"

#define RX_MIN         -32768
#define RX_MAX          32768
#define SERVO_MIN       2000
#define SERVO_MAX       4000

#define K_SHIFT         15
#define K_SCALE         (1LL << K_SHIFT)

#define LIMIT_RX_DEF(x)        ((x) = RxLimit((x), RX_MIN, RX_MAX))
#define LIMIT_RX(x, min, max)  ((x) = RxLimit((x), (min), (max)))

#define RXPT_CNT(x)     (sizeof(x) / sizeof(x[0]))
#define RXCURVE(c)      (c), RXPT_CNT(c)
#define RXPT(x, y)      { (x), (y), 0, 0 }

struct RxCurvePt {
   int x;
   int y;
   int k;
   int d;
};

uint16_t Rx2ServoPWM(int rx);
int RxSetServoPWM(int ch, uint16_t pwm);
uint16_t RxGetServoPWM(int ch);

int RxSetServo(int ch, int rx);

int RxLimit(int val, int min, int max);
int RxInitKd(struct RxCurvePt *cvPt, int elemCnt);
int RxCurve(int x, struct RxCurvePt *cvPt, int elemCnt);

#endif