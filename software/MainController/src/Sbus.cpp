#include <Arduino.h>
#include <stdint.h>

#include "Rx.h"
#include "Sbus.h"
#include "SysTimer.h"

#define INTERFRAME_MIN_GAP    2000
#define FRAME_START_BYTE      0x0f
#define FRAME_END_BYTE        0x00           
#define FRAME_LEN             25
#define FOOTER_POS            24
#define FLAG_CHANNEL_17       0x01
#define FLAG_CHANNEL_18       0x02
#define FLAG_SIGNAL_LOSS      0x04
#define FLAG_FAILSAFE_ACTIVE  0x08
#define NUM_CH                18

#define K_SBUS2RX    (((RX_MAX - RX_MIN) * K_SCALE) / (SBUS_MAX - SBUS_MIN))
#define D_SBUS2RX    (RX_MAX - K_SBUS2RX * SBUS_MAX / K_SCALE)

static uint32_t sTsLastByteUs;
static int sPos;
static int sErr;
static uint8_t frame[FRAME_LEN];
static uint16_t channel[NUM_CH];
static uint16_t sChMin[NUM_CH];
static uint16_t sChMax[NUM_CH];
static int sChMinMaxValid;
static uint32_t sFrameCnt;

struct sbusChannels {
    uint8_t startByte;
    unsigned int ch0 : 11;
    unsigned int ch1 : 11;
    unsigned int ch2 : 11;
    unsigned int ch3 : 11;
    unsigned int ch4 : 11;
    unsigned int ch5 : 11;
    unsigned int ch6 : 11;
    unsigned int ch7 : 11;
    unsigned int ch8 : 11;
    unsigned int ch9 : 11;
    unsigned int ch10 : 11;
    unsigned int ch11 : 11;
    unsigned int ch12 : 11;
    unsigned int ch13 : 11;
    unsigned int ch14 : 11;
    unsigned int ch15 : 11;
    uint8_t flags;
} __attribute__((__packed__));


static eSbusFrameState ProcessFrame(void) {

   struct sbusChannels *sbc = (struct sbusChannels*)frame;
/*
   if (frame[FOOTER_POS != FRAME_END_BYTE]) {
      SysOutStr("Footer Err!");
      return;
   }
  */ 
   channel[0] = sbc->ch0;
   channel[1] = sbc->ch1;
   channel[2] = sbc->ch2;
   channel[3] = sbc->ch3;
   channel[4] = sbc->ch4;
   channel[5] = sbc->ch5;
   channel[6] = sbc->ch6;
   channel[7] = sbc->ch7;
   channel[8] = sbc->ch8;
   channel[9] = sbc->ch9;
   channel[10] = sbc->ch10;
   channel[11] = sbc->ch11;
   channel[12] = sbc->ch12;
   channel[13] = sbc->ch13;
   channel[14] = sbc->ch14;
   channel[15] = sbc->ch15;

   if (sbc->flags & FLAG_CHANNEL_17) {
         channel[16] = SBUS_MAX;
      } else {
         channel[16] = SBUS_MIN;
      }

      if (sbc->flags & FLAG_CHANNEL_18) {
         channel[17] = SBUS_MAX;
      } else {
         channel[17] = SBUS_MIN;
      }

      if (sbc->flags & FLAG_FAILSAFE_ACTIVE) {
         // internal failsafe enabled and rx failsafe flag set
         // RX *should* still be sending valid channel data (repeated), so use it.
         return (eSbusFrameState)(eFrameComplete | eFrameFaileSafe);
      }

      if (sbc->flags & FLAG_SIGNAL_LOSS) {
         // The received data is a repeat of the last valid data so can be considered complete.
      return (eSbusFrameState)(eFrameComplete | eFrameDropped);
   }

   sFrameCnt++;
   /* Update min and max vaules */
   for (int i = 0; i < NUM_CH; i++) {
      if (channel[i] < sChMin[i]) {
         sChMin[i] = channel[i];
      }
      if (channel[i] > sChMax[i]) {
         sChMax[i] = channel[i];
      }
   }
   sChMinMaxValid = 1;

   return eFrameComplete;
}

eSbusFrameState SbusProcess(uint8_t byte) {

   uint32_t delta;
   uint32_t now;

   now = SysGetTimeUs();
   delta = now - sTsLastByteUs;
   sTsLastByteUs = now;

   if (delta > INTERFRAME_MIN_GAP) {
      sPos = 0;
      sErr = 0;
   }

   if (sErr) {
      return eFramePending;
   }

   if (sPos == 0) {
      if (byte == 0x0f) {
         frame[sPos] = byte;
         sPos++;
      }   
   } else if (sPos < FRAME_LEN) {
      frame[sPos] = byte;
      sPos++;
      if (sPos == FRAME_LEN) {
         return ProcessFrame();
      }
   } else if (sPos >= FRAME_LEN) {
      // should not happen => wait for gap and restart...
      Serial.println("Frame Err");
      sPos = 0;
      sErr = 1;
   }
   return eFramePending;
}

uint16_t SbusGetCh(int ch) {
   
   if (ch >= NUM_CH) {
      return 0;
   }

   return channel[ch];
}

/* Rx internal representation is -32768...0...+32768 with center at 0.
   Sbus is about between 240 and 1807 with center at 1023.
   y = k * x + d
   k is mulitplied by K_SCALE
   Division is replaced by shift.
*/
int Sbus2Rx(uint16_t sbus) {

   /* Inverted, so that stick-up and stick-right return positive values */ 
   return -(((sbus * K_SBUS2RX) >> K_SHIFT) + D_SBUS2RX);
}

/* Returns +-32768 */
int SbusGetRx(int ch) {
   
   return Sbus2Rx(SbusGetCh(ch));
}

uint32_t SbusFrameCnt(void) {
   
   return sFrameCnt;
}

void SbusClearMinMax(void) {

   for (int i = 0; i < NUM_CH; i++) {
      sChMin[i] = UINT16_MAX;
      sChMax[i] = 0;
   }
   sChMinMaxValid = 0;
}

/* Returns the max movment since last RxClearMinMax() 
   Can be used to check if sticks were moved. For idle detection. */
int SbusMaxDelta(void) {
  
   uint16_t delta;
   uint16_t maxDelta = 0;

   if (!sChMinMaxValid) {
      return 0;
   }

   for (int i = 0; i < NUM_CH; i++) {
      delta = sChMax[i] - sChMin[i];
      if (delta > maxDelta) {
         maxDelta = delta;
      }
   }

   return maxDelta;
}