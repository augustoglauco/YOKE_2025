#include "joy.h"
#include "AxisCalibration.h"

Joy::Joy(EepromManager& eepromManager)
  : Joystick1(                               // define Joystick parameters
       JOYSTICK_DEFAULT_REPORT_ID,           // ID defined in Joystick.h
       JOYSTICK_TYPE_JOYSTICK,               // type Joystick
       0, 0,                                // Button Count, Hat Switch Count
       true, true, false,                    // X, Y, Z
       false, false, false,                  // Rx, Ry, Rz
       false, false),                         // rudder, throttle)
    eepromManager(eepromManager)   
{
}

void Joy::initializeJoystick()
{
    if(eepromManager.isDataAvailable()==1)
    {
      eepromManager.readDataFromEEPROM(gains, effects, adjForceMax, adjPwmMin, adjPwmMax);
      Serial.println("Read From eeprom"); // Augusto
    }else{
      SetupDefaults();
    }

    SetGains();                           // set gains
    //SetRangeJoystick();                   // set range is calling from main.cpp on setup()
    Joystick1.begin(false);               // start joystick emulation (no auto send updates);
        // Gains* g = Joystick1.getGains();
        // Serial.println(g[1].totalGain);
        // Serial.println(g[1].damperGain);
        // Serial.println(g[1].constantGain);
        // Serial.println(g[1].defaultSpringGain);
        // Serial.println(g[1].frictionGain);
        // Serial.println(g[1].inertiaGain);
        // Serial.println(g[1].rampGain);  
        // Serial.println(g[1].sawtoothdownGain);
        // Serial.println(g[1].sawtoothupGain);
        // Serial.println(g[1].sineGain);
        // Serial.println(g[1].springGain);
        // Serial.println(g[1].squareGain);
        // Serial.println(g[1].triangleGain);
}

void Joy::SetRangeJoystick(AxisConfiguration* rollConfig, AxisConfiguration* pitchConfig) {
    // Serial.print("ROLL MIN: ");
    // Serial.println(rollConfig->iMin);
    // Serial.print("ROLL MAX: ");
    // Serial.println(rollConfig->iMax);
    // Serial.print("PITCH MIN: ");
    // Serial.println(pitchConfig->iMin);
    // Serial.print("PITCH MAX: ");
    // Serial.println(pitchConfig->iMax);
    Joystick1.setXAxisRange(rollConfig->iMin, rollConfig->iMax);
    Joystick1.setYAxisRange(pitchConfig->iMin, pitchConfig->iMax);
};

void Joy::SetupDefaults() {
    for (int i = 0; i < FFB_AXIS_COUNT; ++i) {
        gains[i].totalGain = DEFAULT_GAIN;
        gains[i].constantGain = DEFAULT_GAIN;
        gains[i].rampGain = DEFAULT_GAIN;
        gains[i].squareGain = DEFAULT_GAIN;
        gains[i].sineGain = DEFAULT_GAIN;
        gains[i].triangleGain = DEFAULT_GAIN;
        gains[i].sawtoothdownGain = DEFAULT_GAIN;
        gains[i].sawtoothupGain = DEFAULT_GAIN;
        gains[i].springGain = DEFAULT_GAIN;
        gains[i].damperGain = DEFAULT_GAIN;
        gains[i].inertiaGain = DEFAULT_GAIN;
        gains[i].frictionGain = DEFAULT_FRICTION_GAIN;
        gains[i].defaultSpringGain = 0;
    }
    effects[MEM_ROLL].frictionMaxPositionChange = DEFAULT_FRICTIONMAXPOSITIONCHANGE_ROLL;
    effects[MEM_ROLL].inertiaMaxAcceleration = DEFAULT_INERTIAMAXACCELERATION_ROLL;
    effects[MEM_ROLL].damperMaxVelocity = DEFAULT_DAMPERMAXVELOCITY_ROLL;

    effects[MEM_PITCH].frictionMaxPositionChange = DEFAULT_FRICTIONMAXPOSITIONCHANGE_PITCH;
    effects[MEM_PITCH].inertiaMaxAcceleration = DEFAULT_INERTIAMAXACCELERATION_PITCH;
    effects[MEM_PITCH].damperMaxVelocity = DEFAULT_DAMPERMAXVELOCITY_PITCH;

    adjForceMax[MEM_ROLL]=default_ROLL_FORCE_MAX;
    adjPwmMin[MEM_ROLL]=default_ROLL_PWM_MIN;
    adjPwmMax[MEM_ROLL]=default_ROLL_PWM_MAX;

    adjForceMax[MEM_PITCH]=default_PITCH_FORCE_MAX;
    adjPwmMin[MEM_PITCH]=default_PITCH_PWM_MIN;
    adjPwmMax[MEM_PITCH]=default_PITCH_PWM_MAX;

    //Joystick1.setGains(gains);
}

void Joy::SetAll() {
    Joystick1.setGains(gains);
    Joystick1.setEffectParams(effects);    
}

Gains* Joy::getGains() {
    return gains;
}

void Joy::SetGains(){
    Joystick1.setGains(gains);
}

void Joy::SetGains(Gains gains[]) {
    Joystick1.setGains(gains);
}

EffectParams* Joy::getEffectParams() {
    return effects;
}

void Joy::SetEffectParams() {
    Joystick1.setEffectParams(effects);
}

void Joy::SetEffectParams(EffectParams effects[]) {
    Joystick1.setEffectParams(effects);
}

void Joy::UpdateEffects(bool recalculate, Encoder* counterRoll, Encoder* counterPitch, AxisConfiguration* rollConfig, AxisConfiguration* pitchConfig) {
    int CR = -counterRoll->read();
    int CP = counterPitch->read();

    effects[MEM_ROLL].springMaxPosition = rollConfig->iMax;
    effects[MEM_PITCH].springMaxPosition = pitchConfig->iMax;
    effects[MEM_ROLL].springPosition = CR;
    effects[MEM_PITCH].springPosition = CP;

    unsigned long currentMillis = millis();
    unsigned long diffTime = currentMillis - lastEffectsUpdate;

    if (diffTime > 0 && recalculate) {
        lastEffectsUpdate = currentMillis;
        int16_t positionChangeX = counterRoll->read() - lastX;
        int16_t positionChangeY = counterPitch->read() - lastY;
        int16_t velX = positionChangeX / diffTime;
        int16_t velY = positionChangeY / diffTime;
        int16_t accelX = ((velX - lastVelX) * 10) / diffTime;
        int16_t accelY = ((velY - lastVelY) * 10) / diffTime;

        effects[MEM_ROLL].frictionPositionChange = velX;
        effects[MEM_PITCH].frictionPositionChange = velY;
        effects[MEM_ROLL].inertiaAcceleration = accelX;
        effects[MEM_PITCH].inertiaAcceleration = accelY;
        effects[MEM_ROLL].damperVelocity = velX;
        effects[MEM_PITCH].damperVelocity = velY;

        lastX = counterRoll->read();
        lastY = counterPitch->read();
        lastVelX = velX;
        lastVelY = velY;
        lastAccelX = accelX;
        lastAccelY = accelY;
    } else {
        effects[MEM_ROLL].frictionPositionChange = lastVelX;
        effects[MEM_PITCH].frictionPositionChange = lastVelY;
        effects[MEM_ROLL].inertiaAcceleration = lastAccelX;
        effects[MEM_PITCH].inertiaAcceleration = lastAccelY;
        effects[MEM_ROLL].damperVelocity = lastVelX;
        effects[MEM_PITCH].damperVelocity = lastVelY;
    }

    Joystick1.setXAxis(CR);
    Joystick1.setYAxis(CP);

    // #ifdef SERIAL_DEBUG
    // Serial.print("PITCH: ");
    // Serial.println(CP);
    // Serial.print("ROLL: ");
    // Serial.println(CR);
    // #endif

    Joystick1.setEffectParams(effects);
    Joystick1.getForce(forces);
}

Joystick_& Joy::getJoystick() {
  return Joystick1;
}

// Define the method to return all the data together
// JoyData Joy::getJoyData() {
//     JoyData data;
//     memcpy(data.effects, effects, sizeof(effects));
//     memcpy(data.adjForceMax, adjForceMax, sizeof(adjForceMax));
//     memcpy(data.adjPwmMin, adjPwmMin, sizeof(adjPwmMin));
//     memcpy(data.adjPwmMax, adjPwmMax, sizeof(adjPwmMax));
//     memcpy(data.forces, forces, sizeof(forces));
//     return data;
// }

JoyData Joy::getJoyData() {
    JoyData data;
    for (int i = 0; i < FFB_AXIS_COUNT; ++i) {
        data.effects[i] = &effects[i];
        data.gains[i] = &gains[i];
    }
    for (int i = 0; i < MEM_AXES; ++i) {
        data.adjForceMax[i] = &adjForceMax[i];
        data.adjPwmMin[i] = &adjPwmMin[i];
        data.adjPwmMax[i] = &adjPwmMax[i];
        data.forces[i] = &forces[i];
    }
    return data;
}

void Joy::setJoyData(JoyData& data) {
    for (int i = 0; i < FFB_AXIS_COUNT; ++i) {
        effects[i] = *data.effects[i];
        gains[i] = *data.gains[i];
    }
    for (int i = 0; i < MEM_AXES; ++i) {
        adjForceMax[i] = *data.adjForceMax[i];
        adjPwmMin[i] = *data.adjPwmMin[i];
        adjPwmMax[i] = *data.adjPwmMax[i];
        forces[i] = *data.forces[i];
    }
}


// JoyData Joy::setJoyData(JoyData data) {
//     memcpy(effects, data.effects, sizeof(effects));
//     memcpy(adjForceMax, data.adjForceMax, sizeof(adjForceMax));
//     memcpy(adjPwmMin, data.adjPwmMin, sizeof(adjPwmMin));
//     memcpy(adjPwmMax, data.adjPwmMax, sizeof(adjPwmMax));
//     memcpy(forces, data.forces, sizeof(forces));
//     return data;
// }






