#include "Communication.h"
#include "EepromManager.h"

bool blSerialDebug = false;           // serial debug mode   
byte bSerialIndex = 0;                // serial index
EepromManager eepromM;                // Create a EepromManager instance

Communication::Communication(
   Joy& joy 
  ,AxisConfiguration& rollConfig
  ,AxisConfiguration& pitchConfig
  ,Encoder& counterRoll 
  ,Encoder& counterPitch
  ,byte &roll_speed
  ,byte &pitch_speed 
  ) : 
  joy(joy)
  ,rollConfig(rollConfig)
  ,pitchConfig(pitchConfig)
  ,counterRoll(counterRoll) 
  ,counterPitch(counterPitch)
  ,roll_speed(roll_speed)
  ,pitch_speed(pitch_speed)
  {}        // Initialize serial communication for debugging    

void Communication::begin(long baudRate) {
    Serial.begin(baudRate);   
    //while (!Serial);  // init serial  
}

void Communication::SerialEvent() {
  if (Serial.available()) {
    char cStart = (char)Serial.read(); 
     if (cStart == '!' && Serial.available() > 0) {   // still data available?
      int16_t cmd = Serial.parseInt();                // read command parameter as int value (see command const above)
      if (Serial.available() > 0) {                   // value available?
        int16_t value = Serial.parseInt();            // read value
        if (cmd <= 100) {                             // all above 100 is a write command
          SerialProcessReadCommand(cmd, value);       // process read command
        } else {
          SerialProcessWriteCommand(cmd, value);      // process write command
        }                                             // <=100
      }                                               // available        
    }                                                 //start Flag '!'
  }                                                   // available
  // if serial debug mode is enabled the send data
  if (blSerialDebug == true) {
    SerialWriteStart(SERIAL_CMD_DEBUG_VALUES);    // send start command
    CMD_READ_ALL_VALUES();       // send all values
    CMD_READ_ALL_PARAMS();    // send all parameters
    SerialWriteEnd();     // send end flag
  }
}

/******************************************
  send all parameters to Serial
// *******************************************/
void Communication::CMD_READ_ALL_PARAMS() {

    JoyData joyData = joy.getJoyData();

    // Anpassungswerte für Pitch
    SerialWriteValue(*joyData.adjForceMax[MEM_PITCH]); // 0
    SerialWriteValue(*joyData.adjPwmMin[MEM_PITCH]);   // 1
    SerialWriteValue(*joyData.adjPwmMax[MEM_PITCH]);   // 2

    // Anpassungswerte für Roll
    SerialWriteValue(*joyData.adjForceMax[MEM_ROLL]); // 3
    SerialWriteValue(*joyData.adjPwmMin[MEM_ROLL]);   // 4
    SerialWriteValue(*joyData.adjPwmMax[MEM_ROLL]);   // 5

    // Gains für Roll 
    // SerialWriteValue(*joyData.gains[MEM_ROLL].totalGain);           // 6
    // SerialWriteValue(*joyData.gains[MEM_ROLL].constantGain);         // 7
    // SerialWriteValue(*joyData.gains[MEM_ROLL].rampGain);             // 8
    // SerialWriteValue(*joyData.gains[MEM_ROLL].squareGain);           // 9
    // SerialWriteValue(*joyData.gains[MEM_ROLL].sineGain);             // 10
    // SerialWriteValue(*joyData.gains[MEM_ROLL].triangleGain);         // 11
    // SerialWriteValue(*joyData.gains[MEM_ROLL].sawtoothdownGain);     // 12
    // SerialWriteValue(*joyData.gains[MEM_ROLL].sawtoothupGain);       // 13
    // SerialWriteValue(*joyData.gains[MEM_ROLL].springGain);           // 14
    // SerialWriteValue(*joyData.gains[MEM_ROLL].damperGain);           // 15
    // SerialWriteValue(*joyData.gains[MEM_ROLL].inertiaGain);          // 16
    // SerialWriteValue(*joyData.gains[MEM_ROLL].frictionGain);         // 17
    // SerialWriteValue(*joyData.gains[MEM_ROLL].defaultSpringGain);    // xx exists one more on joystick.h
    
    // // Gains für Roll
     // Gains für Roll
    for (byte i = 0; i < (sizeof(*joyData.gains[MEM_ROLL]) / sizeof((*joyData.gains[MEM_ROLL]).totalGain)) - 1; i++) {
        SerialWriteValue(*((byte*)joyData.gains[MEM_ROLL] + i));
    }

    // Effekte für Roll
    SerialWriteValue(joyData.effects[MEM_ROLL]->frictionMaxPositionChange); // 18
    SerialWriteValue(joyData.effects[MEM_ROLL]->inertiaMaxAcceleration);    // 19
    SerialWriteValue(joyData.effects[MEM_ROLL]->damperMaxVelocity);         // 20

    // SerialWriteValue(*joyData.gains[MEM_PITCH].totalGain);            // 21
    // SerialWriteValue(*joyData.gains[MEM_PITCH].constantGain);         // 22
    // SerialWriteValue(*joyData.gains[MEM_PITCH].rampGain);             // 23    
    // SerialWriteValue(*joyData.gains[MEM_PITCH].squareGain);           // 24
    // SerialWriteValue(*joyData.gains[MEM_PITCH].sineGain);             // 25
    // SerialWriteValue(*joyData.gains[MEM_PITCH].triangleGain);         // 26
    // SerialWriteValue(*joyData.gains[MEM_PITCH].sawtoothdownGain);     // 27
    // SerialWriteValue(*joyData.gains[MEM_PITCH].sawtoothupGain);       // 28
    // SerialWriteValue(*joyData.gains[MEM_PITCH].springGain);           // 29
    // SerialWriteValue(*joyData.gains[MEM_PITCH].damperGain);           // 30
    // SerialWriteValue(*joyData.gains[MEM_PITCH].inertiaGain);          // 31
    // SerialWriteValue(*joyData.gains[MEM_PITCH].frictionGain);         // 32
    // SerialWriteValue(*joyData.gains[MEM_PITCH].defaultSpringGain);    // yy exists one more on joystick.h thas why -1
    
      // Gains für Pitch
    for (byte i = 0; i < (sizeof(*joyData.gains[MEM_PITCH]) / sizeof((*joyData.gains[MEM_PITCH]).totalGain)) - 1; i++) {
        SerialWriteValue(*((byte*)joyData.gains[MEM_PITCH] + i));
    }
    
  // Effekte für Pitch
    SerialWriteValue(joyData.effects[MEM_PITCH]->frictionMaxPositionChange); // 33
    SerialWriteValue(joyData.effects[MEM_PITCH]->inertiaMaxAcceleration);    // 34
    SerialWriteValue(joyData.effects[MEM_PITCH]->damperMaxVelocity);         // 35
}
/******************************************
  send all values to Serial
*******************************************/
void Communication::CMD_READ_ALL_VALUES() {

    JoyData joyData = joy.getJoyData();

    // Alle Buttonzustände
    for (byte channel = 0; channel < 16; channel++) {
        //SerialWriteValue(iYokeButtonPinStates & (1 << channel)); // Index 0 - 15
    }

    // Alle Sensordaten
    for (byte channel = 0; channel < 16; channel++) {
        //SerialWriteValue(iSensorPinStates & (1 << channel)); // Index 16 - 31
    }

    // Roll-Achsenkonfiguration und Messwerte
    SerialWriteValue(rollConfig.iMin); // 32
    SerialWriteValue(rollConfig.iMax); // 33
    SerialWriteValue(counterRoll.read()); // 34
    SerialWriteValue(roll_speed); // 35
    SerialWriteValue(*joyData.forces[MEM_ROLL]); // 36

    // Pitch-Achsenkonfiguration und Messwerte
    SerialWriteValue(pitchConfig.iMin); // 37
    SerialWriteValue(pitchConfig.iMax); // 38
    SerialWriteValue(counterPitch.read()); // 39
    SerialWriteValue(pitch_speed); // 40
    SerialWriteValue(*joyData.forces[MEM_PITCH]); // 41
}

// /******************************************
//   Process Read Command
// *******************************************/
void Communication::SerialProcessReadCommand(int16_t cmd, int16_t val) {
  // checks
  if (cmd == 0) return;

  // choose
  //SerialWriteStart(SERIAL_CMD_READ_ALL_VALUES); // return info to sender
  SerialWriteStart(cmd); // return info to sender
  switch (cmd) {
    case SERIAL_CMD_DEBUG_START:                    // start debug mode
      blSerialDebug = true;                         // set flag
      break;
    case SERIAL_CMD_DEBUG_STOP:                     // stop debug mode
      blSerialDebug = false;                        // reset flag
      break;
    case SERIAL_CMD_READ_ALL_VALUES:                // return all measured values like speed, counter,...
      CMD_READ_ALL_VALUES();                        // send values
      break;
    case SERIAL_CMD_READ_ALL_PARAMS:                // return all parameters like gains and effects
      CMD_READ_ALL_PARAMS();                        // send parameters
      break;
  }
  SerialWriteEnd();                                // end flag
} //SerialProcessReadCommand

// /******************************************
//   Process Write Commands
// *******************************************/
void Communication::SerialProcessWriteCommand(int16_t cmd, int16_t value) {
    if (cmd == 0) return;

    JoyData joyData = joy.getJoyData();

    switch (cmd) {
        case SERIAL_CMD_WRITE_DATA_EEPROM:
            eepromM.writeDataToEEPROM(*joyData.gains, *joyData.effects, *joyData.adjForceMax, *joyData.adjPwmMin, *joyData.adjPwmMax);
            break;
        case SERIAL_CMD_WRITE_EEPROM_CLEAR:
            eepromM.clearEEPROM();
            break;
        default:
            handleGainCommands(cmd, value);
            break;
    }

    joy.SetGains(*joyData.gains);
    joy.SetEffectParams(*joyData.effects);
    SerialWriteStart(cmd);
    SerialWriteEnd();
}

// /****************************************************
//   Writes a Start command to the serial interface
// ****************************************************/
void Communication::SerialWriteStart(byte command) {
  bSerialIndex = 0;       // start index i alway 0 on start command
  Serial.print("!");      // a command is indicated by "!"
  Serial.print(command);  // command constant
  Serial.print("|");      // command and values are seperated with "|"
}

// /***********************************************
//   Writes a End command to the serial interface
// ***********************************************/
void Communication::SerialWriteEnd() {
  Serial.println("");
}

// /******************************************
//   Writes a Value to the serial interface
// *******************************************/
void Communication::SerialWriteValue(int16_t value) {
  Serial.print(bSerialIndex);
  Serial.print(":");          // index and value seperated with ":"
  Serial.print(value);        
  Serial.print(",");          // value and value seperated with ","
  bSerialIndex++;             // increment index
}

// /******************************************
//   Handle Wrrite Gains & Effects Commands
// *******************************************/
void Communication::handleGainCommands(int16_t cmd, int16_t value) {
   
    JoyData joyData = joy.getJoyData();

    uint8_t memIndex = (cmd < SERIAL_CDM_WRITE_PITCH_FORCE_MAX) ? MEM_ROLL : MEM_PITCH;
    switch (cmd) {
        case SERIAL_CDM_WRITE_ROLL_FORCE_MAX:
        case SERIAL_CDM_WRITE_PITCH_FORCE_MAX:
            *joyData.adjForceMax[memIndex] = value;
            break;
        case SERIAL_CDM_WRITE_ROLL_PWM_MIN:
        case SERIAL_CDM_WRITE_PITCH_PWM_MIN:
            *joyData.adjPwmMin[memIndex] = value;
            break;
        case SERIAL_CDM_WRITE_ROLL_PWM_MAX:
        case SERIAL_CDM_WRITE_PITCH_PWM_MAX:
            *joyData.adjPwmMax[memIndex] = value;
            break;
        case SERIAL_CDM_WRITE_ROLL_FRICTION_MAX_POSITION_CHANGE:
        case SERIAL_CDM_WRITE_PITCH_FRICTION_MAX_POSITION_CHANGE:
            joyData.effects[memIndex]->frictionMaxPositionChange = value;
            break;
        case SERIAL_CDM_WRITE_ROLL_INERTIA_MAX_ACCELERATION:
        case SERIAL_CDM_WRITE_PITCH_INERTIA_MAX_ACCELERATION:
            joyData.effects[memIndex]->inertiaMaxAcceleration = value;
            break;
        case SERIAL_CDM_WRITE_ROLL_DAMPER_MAX_VELOCITY:
        case SERIAL_CDM_WRITE_PITCH_DAMPER_MAX_VELOCITY:
            joyData.effects[memIndex]->damperMaxVelocity = value;
            break;
        default:
            setGainCommands(memIndex, cmd, value);
            break;
    }   
}

// // /******************************************
// //   Handle Write Gains Commands
// // *******************************************/
void Communication::setGainCommands(uint8_t memIndex, int16_t cmd, int16_t value) {
    
    Gains& g = joy.getGains()[memIndex];
    switch (cmd) {
        case SERIAL_CDM_WRITE_ROLL_TOTAL_GAIN:
        case SERIAL_CDM_WRITE_PITCH_TOTAL_GAIN:
            g.totalGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_CONSTANT_GAIN:
        case SERIAL_CDM_WRITE_PITCH_CONSTANT_GAIN:
            g.constantGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_RAMP_GAIN:
        case SERIAL_CDM_WRITE_PITCH_RAMP_GAIN:
            g.rampGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_SQUARE_GAIN:
        case SERIAL_CDM_WRITE_PITCH_SQUARE_GAIN:
            g.squareGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_SINE_GAIN:
        case SERIAL_CDM_WRITE_PITCH_SINE_GAIN:
            g.sineGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_TRIANGLE_GAIN:
        case SERIAL_CDM_WRITE_PITCH_TRIANGLE_GAIN:
            g.triangleGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_SAWTOOTH_DOWN_GAIN:
        case SERIAL_CDM_WRITE_PITCH_SAWTOOTH_DOWN_GAIN:
            g.sawtoothdownGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_SAWTOOTH_UP_GAIN:
        case SERIAL_CDM_WRITE_PITCH_SAWTOOTH_UP_GAIN:
            g.sawtoothupGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_SPRING_GAIN:
        case SERIAL_CDM_WRITE_PITCH_SPRING_GAIN:
            g.springGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_DAMPER_GAIN:
        case SERIAL_CDM_WRITE_PITCH_DAMPER_GAIN:
            g.damperGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_INERTIA_GAIN:
        case SERIAL_CDM_WRITE_PITCH_INERTIA_GAIN:
            g.inertiaGain = (byte)value;
            break;
        case SERIAL_CDM_WRITE_ROLL_FRICTION_GAIN:
        case SERIAL_CDM_WRITE_PITCH_FRICTION_GAIN:
            g.frictionGain = (byte)value;
            break;
    }
    //joy.SetGains(&g);
}