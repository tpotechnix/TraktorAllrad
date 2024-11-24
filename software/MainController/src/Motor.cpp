#include <Arduino.h>

#include "driver/mcpwm.h"
#include "Motor.h"

static const int cMotorsOnPin = 14;

static const int cPwmFreq = 16000;
static const int cPwmBits = 10;   // max 14 bits on ESP32-S3
static const int cPwmValMax = (1 << cPwmBits) - 1;
static uint32_t cCapPrescale = 1;
static const int cCapTimerFreq = 80000000;

static const mcpwm_unit_t cMcpwmUnit = MCPWM_UNIT_0;


/* Frontmotor has some problems: Feedback falls sometimes into a mode, where it outputs a high signal
   with short low pulses while the motor is stopped. The low pulses have a duration of 3.5us,
   and a frequency of about 285Hz (would be 34.8 RPM on wheel).
*/
static bool CaptureISR(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data) {

   struct CaptureData *pCap = (struct CaptureData*)user_data;

   // sanity check - should not happen
   if (pCap == 0) {
      return false;
   }

   pCap->diff = edata->cap_value - pCap->lastCap;
   pCap->lastCap = edata->cap_value;

   return false;  // true if taskswitch is needed
}

void MotorInit(struct Motor *motor) {

   pinMode(motor->dirPin, OUTPUT);
   digitalWrite(motor->dirPin, LOW);
   
   ledcSetup(motor->pwmCh, cPwmFreq, cPwmBits);
   ledcAttachPin(motor->pwmPin, motor->pwmCh);
   ledcWrite(motor->pwmCh, motor->pwmLowActive ? cPwmValMax : 0);

   gpio_pullup_en((gpio_num_t)motor->feedbackPin); 
   mcpwm_gpio_init(cMcpwmUnit, motor->capGpioCh, motor->feedbackPin);

   mcpwm_capture_config_t capCfg = {
      .cap_edge = MCPWM_POS_EDGE,
      .cap_prescale = cCapPrescale,
      .capture_cb = CaptureISR,
      .user_data = &motor->capData,
   };
   mcpwm_capture_enable_channel(cMcpwmUnit, motor->capSelectCh, &capCfg);
}

static uint32_t MapSpeed2Pwm(struct Motor *motor, int speed) {

   int pwm;

   speed = abs(speed);

   // todo map via a nice table
   // little hack for now...
   if ((motor->pwmCh == 0) || (motor->pwmCh == 1)) {
      // Backmotors
      pwm = speed * 0.87 + 330;
      if (pwm > 920) {
         pwm = cPwmValMax;
      }
   } else if (motor->pwmCh == 2) {
      // Fromtmotor
      pwm = cPwmValMax - speed * 1.5;
      if (pwm < 0) {
         pwm = 0;
      } else if (pwm > 1000) {
         pwm = cPwmValMax;
      }
   }

   return pwm;
}

// speed from remote (+-1024 ?)
void MotorSetSpeed(struct Motor *motor, int speed) {

   bool dirFwd = true;
   uint32_t pwm;

   if (speed < 0) {
      dirFwd = false;
   }
   if (motor->dirInvert) {
      dirFwd = !dirFwd;
   }

   pwm = MapSpeed2Pwm(motor, speed);

   digitalWrite(motor->dirPin, dirFwd ? HIGH : LOW);
   ledcWrite(motor->pwmCh, pwm);
}

/* Backmotor 102RPM at 11.5V. Frontmotor needs to run 102 * 1.316 = 134RPM.
   At 14V it run 132RPM. Seems to be good enough. All values measured with wheels in the air.
   Backmotor 97RPM at 11.1V */

/* A Frontmotor problem is the blocked detection: If PWM is very low, and motor does not spin
   within some time (seconds?), it stops working. Change of direction is needed to reset this.
   Frontmotor seems to have 280RPM no load speed at 12V. Not the 320RPM as stated on label. */

float MotorGetRpm(struct Motor *motor) {

   float dur;
   float rpm;

   dur = motor->capData.diff / (float)cCapTimerFreq;
   rpm = 60.0 / dur / (float)motor->pulsesPerMotorRev;
   rpm = rpm * cCapPrescale;
   rpm = rpm / motor->gearRatio;

   return rpm;
}

// in mm/s
float MotorGetGroundSpd(struct Motor *motor) {
   
   return MotorGetRpm(motor) / 60.0 * motor->wheelDiameter;
}