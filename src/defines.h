#pragma once
#ifndef DEFINES_H
   #define DEFINES_H
   #include <Arduino.h>   
   
   #define SERIAL_BAUD 115200  // Communication Speed
   #define SERIAL_DEBUG 1
   #define BUZZER_PIN MISO //12

   /*****************************
    Memory array positions for Effects
   ****************************/
   #define MEM_ROLL  0
   #define MEM_PITCH  1

   // Pitch Motordriver pins
   #define PITCH_EN  8 //11 
   #define PITCH_U_PWM 9 //6
   #define PITCH_D_PWM 10 //13

   // Roll Motordriver pins
   #define ROLL_EN 7 //8
   #define ROLL_R_PWM 5 //9
   #define ROLL_L_PWM 6 //6 //10

   // multiplexer for buttons
   #define MUX_S0 A0
   #define MUX_S1 A1
   #define MUX_S2 A2
   #define MUX_S3 A3

   // // Multiplexer Yoke Buttons
   // #define MUX_EN_YOKE 5
   // #define MUX_SIGNAL_YOKE A5

   // Multiplexer Adjustemts for Calib Button, Force Potis, End Switches
   #define MUX_EN_INPUT 15 //4
   const byte MUX_EN_INPUT_ADD = B00000010;
   #define MUX_SIGNAL_INPUT A6 //A4

   // Encoder PINS Roll
   #define ROLL1_PIN 1
   #define ROLL2_PIN 0

   #define PITCH1_PIN 2 //2
   #define PITCH2_PIN 3 //3

   // Adjustments array positions
   #define ADJ_ENDSWITCH_PITCH_DOWN 0
   #define ADJ_ENDSWITCH_PITCH_UP 1
   #define ADJ_ENDSWITCH_ROLL_LEFT 2
   #define ADJ_ENDSWITCH_ROLL_RIGHT 3
   #define ADJ_CALIBRATION_BUTTON 4
   
   // Default vaules for gains and effect if nothing saved into eeprom
   #define DEFAULT_GAIN 100
   #define DEFAULT_FRICTION_GAIN 25

   #define DEFAULT_FRICTIONMAXPOSITIONCHANGE_ROLL 125;
   #define DEFAULT_INERTIAMAXACCELERATION_ROLL 100;
   #define DEFAULT_DAMPERMAXVELOCITY_ROLL 350;

   #define DEFAULT_FRICTIONMAXPOSITIONCHANGE_PITCH 125;
   #define DEFAULT_INERTIAMAXACCELERATION_PITCH 100;
   #define DEFAULT_DAMPERMAXVELOCITY_PITCH 350;

   #define default_PITCH_FORCE_MAX 10000;
   #define default_PITCH_PWM_MAX 150;
   #define default_PITCH_PWM_MIN 40;

   #define default_ROLL_FORCE_MAX 10000;
   #define default_ROLL_PWM_MAX 150;
   #define default_ROLL_PWM_MIN 40;

   /******************************************
      Beep
   *******************************************/
   #define BEEP_SHORT_TONE 200
   #define BEEP_LONG_TONE 600
   #define BEEP_CODE_FREQUENCY 1000
   #define BEEP_CODE_DELAY 1000
   #define BEEP_CODE_COUNT 3 //start with 0

#endif