#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "joy.h"
#include "multiplexer.h"

/******************************************
      Communication Command Constants
   *******************************************/
   //Debug
   #define SERIAL_CMD_DEBUG_START 1
   #define SERIAL_CMD_DEBUG_STOP 2
   #define SERIAL_CMD_DEBUG_STATUS 3
   #define SERIAL_CMD_DEBUG_VALUES 4

   #define SERIAL_CMD_DEBUG_FORCE_VALUES 8

   // read
   #define SERIAL_CMD_READ_ALL_PARAMS 10
   #define SERIAL_CMD_READ_ALL_VALUES 20

   // Write Roll
   #define SERIAL_CDM_WRITE_ROLL_FORCE_MAX 101
   #define SERIAL_CDM_WRITE_ROLL_PWM_MIN 102
   #define SERIAL_CDM_WRITE_ROLL_PWM_MAX 103

   #define SERIAL_CDM_WRITE_ROLL_FRICTION_MAX_POSITION_CHANGE 104
   #define SERIAL_CDM_WRITE_ROLL_INERTIA_MAX_ACCELERATION 105
   #define SERIAL_CDM_WRITE_ROLL_DAMPER_MAX_VELOCITY 106

   #define SERIAL_CDM_WRITE_ROLL_TOTAL_GAIN 107
   #define SERIAL_CDM_WRITE_ROLL_CONSTANT_GAIN 108
   #define SERIAL_CDM_WRITE_ROLL_RAMP_GAIN 109
   #define SERIAL_CDM_WRITE_ROLL_SQUARE_GAIN 110
   #define SERIAL_CDM_WRITE_ROLL_SINE_GAIN 111
   #define SERIAL_CDM_WRITE_ROLL_TRIANGLE_GAIN 112
   #define SERIAL_CDM_WRITE_ROLL_SAWTOOTH_DOWN_GAIN 113
   #define SERIAL_CDM_WRITE_ROLL_SAWTOOTH_UP_GAIN 114
   #define SERIAL_CDM_WRITE_ROLL_SPRING_GAIN 115
   #define SERIAL_CDM_WRITE_ROLL_DAMPER_GAIN 116
   #define SERIAL_CDM_WRITE_ROLL_INERTIA_GAIN 117
   #define SERIAL_CDM_WRITE_ROLL_FRICTION_GAIN 118

   // write pitch
   #define SERIAL_CDM_WRITE_PITCH_FORCE_MAX 119
   #define SERIAL_CDM_WRITE_PITCH_PWM_MIN 120
   #define SERIAL_CDM_WRITE_PITCH_PWM_MAX 121

   #define SERIAL_CDM_WRITE_PITCH_FRICTION_MAX_POSITION_CHANGE 122
   #define SERIAL_CDM_WRITE_PITCH_INERTIA_MAX_ACCELERATION 123
   #define SERIAL_CDM_WRITE_PITCH_DAMPER_MAX_VELOCITY 124

   #define SERIAL_CDM_WRITE_PITCH_TOTAL_GAIN 125
   #define SERIAL_CDM_WRITE_PITCH_CONSTANT_GAIN 126
   #define SERIAL_CDM_WRITE_PITCH_RAMP_GAIN 127
   #define SERIAL_CDM_WRITE_PITCH_SQUARE_GAIN 128
   #define SERIAL_CDM_WRITE_PITCH_SINE_GAIN 129
   #define SERIAL_CDM_WRITE_PITCH_TRIANGLE_GAIN 130
   #define SERIAL_CDM_WRITE_PITCH_SAWTOOTH_DOWN_GAIN 131
   #define SERIAL_CDM_WRITE_PITCH_SAWTOOTH_UP_GAIN 132
   #define SERIAL_CDM_WRITE_PITCH_SPRING_GAIN 133
   #define SERIAL_CDM_WRITE_PITCH_DAMPER_GAIN 134
   #define SERIAL_CDM_WRITE_PITCH_INERTIA_GAIN 135
   #define SERIAL_CDM_WRITE_PITCH_FRICTION_GAIN 136

   // eeprom
   #define SERIAL_CMD_WRITE_DATA_EEPROM 250
   #define SERIAL_CMD_WRITE_EEPROM_CLEAR 251

class Communication {
  public:
    Communication(
       Joy &joy
      ,Multiplexer &multiplexer
      ,AxisConfiguration& rollConfig
      ,AxisConfiguration& pitchConfig
      ,Encoder &counterRoll 
      ,Encoder &counterPitch
      ,byte &roll_speed
      ,byte &pitch_speed 
      );
   
    void begin(long baudRate);
    void SerialEvent();
    void SerialProcessReadCommand(int16_t cmd, int16_t val);
    void SerialProcessWriteCommand(int16_t cmd, int16_t val);
    
  private:    

    Joy& joy; // reference to joy object
    Multiplexer& multiplexer; // reference to multiplexer object
    long baudRate; // Baud rate for serial communication
    AxisConfiguration& rollConfig; // reference to rollConfig object
    AxisConfiguration& pitchConfig; // reference to pitchConfig object
    Encoder& counterRoll; // reference to counterRoll object
    Encoder& counterPitch; // reference to pitchRoll object
    byte& roll_speed; // reference to roll_speed object
    byte& pitch_speed; // reference to pitch_speed object 
    void SerialWriteStart(byte command);    
    void SerialWriteEnd();  
    void handleGainCommands(int16_t cmd, int16_t value);
    void setGainCommands(uint8_t memIndex, int16_t cmd, int16_t value);
    void CMD_READ_ALL_VALUES();
    void CMD_READ_ALL_PARAMS();
    void SerialWriteValue(int16_t value);
};

#endif


