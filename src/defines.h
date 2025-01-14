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
   #define MEM_AXES 2

   // Motordriver Enable pin
   #define ROLL_PITCH_EN  7 //11 

   // Pitch Motordriver pins
   #define PITCH_U_PWM 9 //6
   #define PITCH_D_PWM 10 //13

   // Roll Motordriver pins
   #define ROLL_R_PWM 5 //9
   #define ROLL_L_PWM 6 //6 //10

   // multiplexer for buttons
   #define MUX_S0 A0
   #define MUX_S1 A1
   #define MUX_S2 A2
   #define MUX_S3 A3

   // // Multiplexer Yoke Buttons
   #define MUX_EN_YOKE 16 // MOSI
   #define MUX_SIGNAL_YOKE 8
   const byte MUX1_EN_INPUT_ADD = B00000100; 

   // Multiplexer Adjustemts for Calib Button, Force Potis, End Switches
   #define MUX_EN_INPUT 15 //4
   #define MUX_SIGNAL_INPUT A6 //A4
   const byte MUX2_EN_INPUT_ADD = B00000010; 
   
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
   #define ADJ_MOTOR_POWER 5
   
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

#endif