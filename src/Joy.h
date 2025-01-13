#pragma once
#ifndef JOYSTICK_SETUP_H
#define JOYSTICK_SETUP_H
#include <Arduino.h>
#include <Joystick.h>
#include "defines.h"
#include "EepromManager.h"
#include "AxisCalibration.h"

struct JoyData {
    EffectParams* effects[FFB_AXIS_COUNT];
    Gains* gains[FFB_AXIS_COUNT];    
    int16_t* adjForceMax[MEM_AXES];
    byte* adjPwmMin[MEM_AXES];
    byte* adjPwmMax[MEM_AXES];
    int16_t* forces[MEM_AXES];
};

class Joy {
    private:
        Joystick_ Joystick1;         // define Joystick                        
        EepromManager& eepromManager;

        EffectParams effects[FFB_AXIS_COUNT];   // define effects     
        Gains gains[FFB_AXIS_COUNT];
        int16_t adjForceMax[MEM_AXES];          // stored max adjusted force 
        byte adjPwmMin[MEM_AXES];               // stored start pwm (motor power) on force != 0
        byte adjPwmMax[MEM_AXES];               // stored max pwm (motor power) onmax force
        int16_t forces[MEM_AXES];               // current forces
        
        unsigned long lastEffectsUpdate = 0;    // count millis for next effect calculation
        int16_t lastX;                          // X value from last loop
        int16_t lastY;                          // Y value from last loop
        int16_t lastVelX;                       // Velocity X value from last loop
        int16_t lastVelY;                       // Velocity y value from last loop
        int16_t lastAccelX;                     // Acceleration X value from last loop
        int16_t lastAccelY;                     // Acceleration X value from last loop

        //AxisConfiguration rollConfig = {false, -256, 256, false, false}; // Define rollConfig
        //AxisConfiguration pitchConfig = {false, -256, 256, false, false}; // Define pitchConfig

    public: 
        Joy(EepromManager& eepromManager);

        void initializeJoystick();

        void SetupDefaults();

        void SetRangeJoystick(AxisConfiguration *rollConfig, AxisConfiguration *pitchConfig);

        void SetGains();

        void SetAll();

        void SetGains(Gains gains[]);

        void UpdateEffects(bool recalculate, Encoder *counterRoll, Encoder *counterPitch, AxisConfiguration *rollConfig, AxisConfiguration *pitchConfig);

        EffectParams* getEffectParams();

        void SetEffectParams();

        void SetEffectParams(EffectParams effects[]);

        Gains* getGains();

        Joystick_& getJoystick() ;

        JoyData getJoyData();
        
        void setJoyData(JoyData& data);
};

#endif