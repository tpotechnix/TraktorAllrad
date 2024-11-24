#include "Rx.h"

#define K_RX2SERVO      (((SERVO_MAX - SERVO_MIN) * K_SCALE) / (RX_MAX - RX_MIN))
#define D_RX2SERVO      (SERVO_MAX - K_RX2SERVO * RX_MAX / K_SCALE)

/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest integer. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
#define ROUND_DIV(x, divisor)(          \
{                           \
    typeof(x) __x = x;              \
    typeof(divisor) __d = divisor;          \
    (((typeof(x))-1) > 0 ||             \
     ((typeof(divisor))-1) > 0 || (__x) > 0) ?  \
        (((__x) + ((__d) / 2)) / (__d)) :   \
        (((__x) - ((__d) / 2)) / (__d));    \
}                           \
)

uint16_t Rx2ServoPWM(int rx) {

   return ((rx * K_RX2SERVO + K_SCALE / 2) >> K_SHIFT) + D_RX2SERVO;
}

int RxSetServoPWM(int ch, uint16_t pwm) {

   switch (ch) {
      case 0: /* todo */; break;
      case 1: ; break;
      case 2: ; break;
      case 3: ; break;
      default: 
         return -1;
   }

   return 0;
}

uint16_t RxGetServoPWM(int ch) {

   switch (ch) {
      case 0: return 0 /* todo */;
      case 1: return 0;
      case 2: return 0;
      case 3: return 0;
      default: 
         return 0;
   }
}

int RxSetServo(int ch, int rx) {
   
   return RxSetServoPWM(ch, Rx2ServoPWM(rx));
}

int RxLimit(int val, int min, int max) {

   if (val < min) {
      return min;
   } if (val > max) {
      return max;
   }
   return val;
}

int RxInitKd(struct RxCurvePt *cvPt, int elemCnt) {

   int i;
   int k;
   int d;

   if (elemCnt < 2)
      return -1;

   for (i = 0; i < elemCnt - 1; i++) {
       k = ROUND_DIV(((long long)cvPt[i + 1].y - cvPt[i].y) * K_SCALE,
                     cvPt[i + 1].x - cvPt[i].x);
       d = cvPt[i].y - ROUND_DIV(k * cvPt[i].x, K_SCALE);
       cvPt[i].k = k;
       cvPt[i].d = d;
   }

   return 0;
}

int RxCurve(int x, struct RxCurvePt *cvPt, int elemCnt) {

   int i;
   int y;
   int slot = 0;

   if (elemCnt < 2)
      return 0;

   if (x <= cvPt[0].x) {
      /* very low value */
      slot = 0;
   } else if (x >= cvPt[elemCnt - 1].x) {
      /* very high value */
      slot = elemCnt - 2;
   } else {
      /* Search for correct slot */
      for (i = 0; i < elemCnt - 1; i++) {
         if (cvPt[i + 1].x > x) {
            slot = i;
            break;
         }
      }
   }
   
   /* y = k * x + d */
   y = ((x * cvPt[slot].k + K_SCALE / 2) >> K_SHIFT) + cvPt[slot].d;

   //printf("x = %d, slot = %d, x = %d, x = %d, k = %d, d = %d, y = %d\n", x, slot, cvPt[slot].x, cvPt[slot + 1].x, cvPt[slot].k, cvPt[slot].d, y);
   
   return y;
}
