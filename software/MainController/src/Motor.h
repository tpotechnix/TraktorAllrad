#ifndef _MOTOR_H_
#define _MOTOR_H

#include "driver/mcpwm.h"
#include <stdint.h>

struct CaptureData {
   uint32_t lastCap;
   uint32_t diff;
};

struct Motor {
   const int wheelDiameter;
   const float gearRatio;
   const int pulsesPerMotorRev;
   const int pwmPin;
   const int dirPin;
   const int feedbackPin;
   const bool pwmLowActive;
   const bool dirInvert;
   // todo add linearisation data
   const uint8_t pwmCh;     // unique number 0..7
   const mcpwm_io_signals_t capGpioCh;
   const mcpwm_capture_signal_t capSelectCh;
   struct CaptureData capData;
};

void MotorInit(struct Motor *motor);
void MotorSetSpeed(struct Motor *motor, int speed);
float MotorGetRpm(struct Motor *motor);
float MotorGetGroundSpd(struct Motor *motor);

#endif /* _MOTOR_H_*/