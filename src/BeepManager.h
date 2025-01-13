#pragma once
#ifndef BEEPMANAGER_H
#define BEEPMANAGER_H

#include <Arduino.h>

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
  private:
    int buzzerPin;

    // create tone
    void manualTone(int frequency, int duration);

    // beep with drequency and duration
    void beep(int duration, int frequency);

    // beep code for roll axis
    // void calibrationBeepRoll();

    // beep code for pitch axis
    //void calibrationBeepPitch();

    // switch between axis
    //void calibrationBeepAxis(bool isRoll);
};

#endif
