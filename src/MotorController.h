#pragma once
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>    // Include the Arduino library
#include "Multiplexer.h"    // Include the Multiplexer class header file
#include "defines.h"    // Include the Joystick class header file

class MotorController {
private:
    Multiplexer& mux;
    byte& roll_speed;
    byte& pitch_speed;

    void MoveMotorByForce(byte &rSpeed, bool blEndSwitch, byte pinLPWM, byte pinRPWM, int16_t gForce, int forceMax, byte pwmMin, byte pwmMax);

public:
    MotorController(Multiplexer& multiplexer, byte& roll_speed, byte& pitch_speed);

    //void ArduinoSetup();
    void EnableMotors();
    void DisableMotors();
    void PrepareMotors(int16_t forces[], int16_t forceMax[], byte pwmMin[], byte pwmMax[]);
};

#endif