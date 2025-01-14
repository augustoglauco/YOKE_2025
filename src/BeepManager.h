#pragma once
#ifndef BEEPMANAGER_H
#define BEEPMANAGER_H

#include <Arduino.h>

/******************************************
  Beep
*******************************************/
#define BEEP_SHORT_TONE 200
#define BEEP_LONG_TONE 600
#define BEEP_CODE_FREQUENCY 1000
#define BEEP_CODE_DELAY 1000
#define BEEP_CODE_COUNT 3 //start with 0

class BeepManager {
  public:
    // Constructor
    BeepManager(int pin);

    // System Start
    void systemStart();

    // Start Calibration
    void calibrationStart();

    // Calibration Successful
    void calibrationSuccess();

    // Calibration Error
    void calibrationError();

    // Calibration Movement Timeout on Axis
    void calibrationTimeoutMotor();

    // General calibration Timeout
    void calibrationTimeoutGeneral();

    // Wrong motor direction
    void calibrationMotorInverted();

    // wrong encoder direction
    void calibrationEncoderInverted();
    
    void NoMotorPower();

    void calibrationBeepAxis(bool isRoll);

private:
    int buzzerPin;

    // create tone
    void manualTone(int frequency, int duration);

    // beep with drequency and duration
    void beep(int duration, int frequency);

    // beep code for roll axis
     void calibrationBeepRoll();

    // beep code for pitch axis
    void calibrationBeepPitch();


};

#endif
