#include <Arduino.h>

#include <HardwareSerial.h>

#include "Sbus.h"
#include "SysTimer.h"

#include "driver/mcpwm.h"

#include "Motor.h"

// Pins
const int cLedPin = 46;
const int cMotorsOnPin = 14;
const int cMotorsCurrentPin = 13;
const int cVBattOffPin = 1;      // ADC read voltage, drive low to poweroff
const int cSbusInPin = 2;

const int cSvoFrontSignalPin = 6;
const int cSvoFrontPosPin = 8;
const int cSvoBackSignalPin = 17;
const int cSvoBackPosPin = 15;
const int cSvoSteerSignalPin = 16;

const int cConnFrontSvoSignalPin = 48;
const int cConnFrontSbusOutPin = 47;
const int cConnBackSvoSignalPin = 31;
const int cConnBackSbusOutPin = 39;

const int cAudioBclkPin = 40;
const int cAudioLrcPin = 41;
const int cAudioDoutPin = 42;

const int cI2C0SclPin = 4;
const int cI2C0SdaPin = 5;
const int cI2C1SclPin = 19;   // mux with USB
const int cI2C1SdaPin = 20;   // mux with USB

const float SVO_MID_PERCENT = 8.3;

static int cDebugBaudrate = 115200;
static int sVBattAdcFact = 4300;   // 1/1000, volatge divider 33k + 10k

static int cRxChannelSteer = 0;
static int cRxChannelSpeed = 2;

static HardwareSerial sSbusSerial(1);     // UART1

static struct Motor sMotorRight = {
   .wheelDiameter = 125,
   .gearRatio = 56,
   .pulsesPerMotorRev = 9,
   .pwmPin = 9,
   .dirPin = 10,
   .feedbackPin = 3,
   .pwmLowActive = false,
   .dirInvert = true,
   .pwmCh = 0,
   .capGpioCh = MCPWM_CAP_0,
   .capSelectCh = MCPWM_SELECT_CAP0,
};
static struct Motor sMotorLeft = {
   .wheelDiameter = 125,
   .gearRatio = 56,
   .pulsesPerMotorRev = 9,
   .pwmPin = 11,
   .dirPin = 45,
   .feedbackPin = 18,
   .pwmLowActive = false,
   .dirInvert = false,
   .pwmCh = 1,
   .capGpioCh = MCPWM_CAP_1,
   .capSelectCh = MCPWM_SELECT_CAP1,
};
static struct Motor sMotorFront = {
   .wheelDiameter = 95,
   .gearRatio = 21.3 * 2.533,  // gearmotor and differential
   .pulsesPerMotorRev = 9,
   .pwmPin = 7,
   .dirPin = 12,
   .feedbackPin = 21,
   .pwmLowActive = true,
   .dirInvert = false,
   .pwmCh = 2,
   .capGpioCh= MCPWM_CAP_2,
   .capSelectCh = MCPWM_SELECT_CAP2,
};


static void SvoPwmInit(void) {
   
   mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, cSvoSteerSignalPin);

   mcpwm_config_t pwm_config;
   pwm_config.frequency = 50;    //frequency
   pwm_config.cmpr_a = SVO_MID_PERCENT;       //duty cycle of PWMxA
   pwm_config.cmpr_b = 0.0;       //duty cycle of PWMxb
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void setup() {
   
   pinMode(cLedPin, OUTPUT);
   digitalWrite(cLedPin, HIGH);

   Serial.begin(cDebugBaudrate);
   Serial.println("TraktorAllrad start");

   // UART receives Sbus and transmits Sbus to backward connecor
   // todo: UART for front Sbus connector. What to do with RX of that UART??
   sSbusSerial.begin(100000, SERIAL_8N1, cSbusInPin, cConnBackSbusOutPin, true /* invert */);

   MotorInit(&sMotorRight);
   MotorInit(&sMotorLeft);
   MotorInit(&sMotorFront);

   // supply power to motors
   pinMode(cMotorsOnPin, OUTPUT);
   digitalWrite(cMotorsOnPin, HIGH);

   SvoPwmInit();
}

static void PowerOff() {
   
   Serial.println("PowerOff!");
   pinMode(cVBattOffPin, OUTPUT);
   digitalWrite(cVBattOffPin, LOW);
   while(1) {
      Serial.print("x");
   }
}

static void MotorsUpdate() {

   int spd = SbusGetCh(cRxChannelSpeed) - 1024;

   MotorSetSpeed(&sMotorFront, spd);
   MotorSetSpeed(&sMotorRight, spd);
   MotorSetSpeed(&sMotorLeft, spd);
}

static void SvoUpdate() {

   float svoFlt;  // 5% .. 7,5% .. 10% !?!?

   svoFlt = SVO_MID_PERCENT + (SbusGetCh(cRxChannelSteer) - 1024) / 350.0;

 //  Serial.printf("Svo: %.2f\n", svoFlt);

   mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, svoFlt);
}

static void ApplUpdate() {
   
   static int sCnt = 0;

   MotorsUpdate();
   SvoUpdate();

   if ((sCnt % 100) == 0) {
      Serial.printf("--- SBUS: Speed: %d\n", SbusGetCh(cRxChannelSpeed));

      Serial.printf("Front: RPM: %.2f, GroundSpeed: %.2f\n", MotorGetRpm(&sMotorFront), MotorGetGroundSpd(&sMotorFront)); 
      Serial.printf("Right: RPM: %.2f, GroundSpeed: %.2f\n", MotorGetRpm(&sMotorRight), MotorGetGroundSpd(&sMotorRight)); 
      Serial.printf("Left:  RPM: %.2f, GroundSpeed: %.2f\n", MotorGetRpm(&sMotorLeft), MotorGetGroundSpd(&sMotorLeft)); 
   }

   sCnt++;
}

void loop() {

   if ((SysGetTimeMs() % 100) == 0) {
      digitalWrite(cLedPin, HIGH);
   } else {
      digitalWrite(cLedPin, LOW);
   }

   while (sSbusSerial.available() > 0) {
      eSbusFrameState sbusState;
         
      sbusState = SbusProcess(sSbusSerial.read());
      if (sbusState == eFrameComplete) {
         ApplUpdate();
      }           
   }
/*
   int start = SysGetTimeUs();
   int val = analogReadMilliVolts(cVBattOffPin);   // todo ev. auf nonblocking umbauen
   int vBatt = val * sVBattAdcFact / 1000;
   int stop = SysGetTimeUs();
   //Serial.printf("Time: %dus", stop-start);
   Serial.printf("val: %d, volt: %d\n", val, vBatt);

   if (vBatt < 10000) {
      PowerOff();
   }
   */

}


#if 0

HardwareSerial MySerial(1);   // UART1
const int MySerialRX = 18;
const int MySerialTX = 17;  // ??

const int cPwmCh = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int cPwmFreq = 50;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int cPwmRes = 14; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits 

const int cPwmPin = 4;
const int cLedPin = 46;

#define SPD_DIR    9

#define GPIO_PWM0A_OUT  10

static void configMCPWM(void) {
   
   mcpwm_pin_config_t pin_config = {
      .mcpwm0a_out_num = GPIO_PWM0A_OUT,
/*        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
        .mcpwm1a_out_num = GPIO_PWM1A_OUT,
        .mcpwm1b_out_num = GPIO_PWM1B_OUT,
        .mcpwm2a_out_num = GPIO_PWM2A_OUT,
        .mcpwm2b_out_num = GPIO_PWM2B_OUT,*/
   };
   mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
   // or: mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);

   mcpwm_config_t pwm_config;
   pwm_config.frequency = 10000;    //frequency
   pwm_config.cmpr_a = 0.0;       //duty cycle of PWMxA
   pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
}


void setup() {
   pinMode (LED_BUILTIN, OUTPUT);
   pinMode(cLedPin, OUTPUT);
   pinMode(SPD_DIR, OUTPUT);

   Serial.begin(115200);
   Serial.println("Start");
   
   while(1) {
      Serial.print("X");
      digitalWrite(cLedPin, HIGH);
      delay(200);
      digitalWrite(cLedPin, LOW);
      delay(200);
   }


	// initialize the Serial to the pins
   MySerial.begin(100000, SERIAL_8N1, MySerialRX, MySerialTX, true);

   // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
   // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
   ledcSetup(cPwmCh, cPwmFreq, cPwmRes);

   // ledcAttachPin(uint8_t pin, uint8_t channel);
   ledcAttachPin(cPwmPin, cPwmCh);
   ledcWrite(cPwmCh, 820);

   configMCPWM();

   //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 10.0);
}


static void ApplUpdate() {

   static int sCnt = 0;
/*
   if ((sCnt % 100) == 0) {
      for (int i = 0; i < 18; i++) {
         Serial.printf("%d: %d\n", i, SbusGetCh(i));
      }
      Serial.println();
   }
*/
   if (1) {
      static int sValOld;
      static unsigned long sDtOld;
      unsigned long dt;
      int val = SbusGetCh(0);

      ledcWrite(cPwmCh, 200 + val / 2);
      dt = SysGetTimeUs();
      //Serial.printf("dt: %dus, val: %d: delta: %d\n", dt - sDtOld, val, val - sValOld);
      sValOld = val;
      sDtOld = dt;

      int valSpd = SbusGetCh(2);
      int spd = valSpd - 1022;  // (+-700)

      if (spd < 0) {
         digitalWrite(SPD_DIR, LOW);
         spd = -spd;
      } else {
         digitalWrite(SPD_DIR, HIGH);
      }
      float pwmFlt = (float)spd / 7.0;
      Serial.printf("flt: %.1f\n", pwmFlt);
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, pwmFlt);

   }



   sCnt++;
}

static int sCnt = 0;
static unsigned int pwm = 820;

void loop() {

   //ledcWrite(cPwmCh, pwm);

   if ((SysGetTimeMs() % 100) == 0) {
      digitalWrite(LED_BUILTIN, HIGH);
   } else {
      digitalWrite(LED_BUILTIN, LOW);
   }

   while (MySerial.available() > 0) {
      eSbusFrameState sbusState;
         
      sbusState = SbusProcess(MySerial.read());
      if (sbusState == eFrameComplete) {
         ApplUpdate();
      }           
   }
/*
   if ((SysGetTimeMs() % 70) == 0) {
      ledcWrite(cPwmCh, pwm);
      pwm += 1;
      Serial.printf("pwm: %d\n", pwm);
   }

   if (pwm > 1600) {
      pwm = 820;
   }
*/
/*
   static float sPos = 5;
   static int sLastMillis = 0;
   unsigned long m = millis();
   if (m != sLastMillis) {
      mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, sPos);
      sPos += 0.001;
      sLastMillis = m;
      if (sPos > 10.0) {
         sPos = 3.0;
      }
   }
   */
}

#endif

#if 0
   char b;
   int nr;
   eSbusFrameState sbusState;
   uint32_t timeMs;
   static uint32_t sLastTimeMs;

   halstuffInit();

   InitUart();
   InitTimers();
   
#if 1  /* Can be removed during debug, to avoid having to hold power-on btn */
   /* Turn off power. If ON-Button is still pressed, than power stays on.
      If ON-Button is not pressed, we stay turned off. */
   PowerOff();
   SysOutStr("Power on delay...\n\r");
   HAL_Delay(400);
   SysOutStr("done\n\r");
   PowerOn();
#endif

   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

   /* Turn on green LED */
   HAL_GPIO_WritePin(LED_PPM_GPIO_Port, LED_PPM_Pin, 1);
   
   SbusClearMinMax();

   while (1) {
      timeMs = SysGetTimeMs();

      nr = SioRead(sSbusHdl, &b, 1);
      if (nr == 1) {
         sbusState = SbusProcess(b);
         if (sbusState == eFrameComplete) {
            ApplUpdate();
         }
      }

      if (timeMs != sLastTimeMs) {
         sLastTimeMs = timeMs;
         CheckVoltage();
         CheckIdle();
      }
   }
}
#endif