#include "Multiplexer.h"
#include "defines.h"

// Constructor to initialize the Joystick pointer and end switch state pointers
Multiplexer::Multiplexer(Joystick_* joystickPtr) {
    this->joy = joystickPtr;
}

// Method to read the multiplexer and update end switches
void Multiplexer::ReadMux() {
  iYokeButtonPinStates = 0;
  iSensorPinStates = 0;

  // for every 16 imput lines of a multiplexer
  for (byte x = 0; x < 16; x++) {
    for (int i = 0; i < 4; i++) {
        PORTF = (x & (1 << i)) ? (PORTF | (1 << (7 - i))) : (PORTF & ~(1 << (7 - i)));  // set the address lines
    }

    // enable mux 1
    PORTB &= ~MUX1_EN_INPUT_ADD; // Digital Pin 16- PortB2 - MOSI
    delayMicroseconds(1);       // wait for capacitors of mux to react
    iYokeButtonPinStates |= digitalRead(MUX_SIGNAL_YOKE) << x; // Read value from the first multiplexer
    PORTB |= MUX1_EN_INPUT_ADD; // disable mux1 -Digital Pin 5 - PortB2 - MOSI

    //enable mux 2
    PORTB &= ~MUX2_EN_INPUT_ADD; // SCLK - PB01 - 
    //digitalWrite(MUX_EN_INPUT, LOW);
    delayMicroseconds(1); // wait for capacitors of mux to react    
    iSensorPinStates |= digitalRead(MUX_SIGNAL_INPUT) << x; // Read value from the second multiplexer
    PORTB |= MUX2_EN_INPUT_ADD;    // disblae mux 2
  }  //for

  //Check end switches
  blEndSwitchRollLeft=(iSensorPinStates & (1 << ADJ_ENDSWITCH_ROLL_LEFT)) == 0;
  blEndSwitchRollRight=(iSensorPinStates & (1 << ADJ_ENDSWITCH_ROLL_RIGHT)) == 0;
  blEndSwitchPitchUp=(iSensorPinStates & (1 << ADJ_ENDSWITCH_PITCH_UP)) == 0;
  blEndSwitchPitchDown=(iSensorPinStates & (1 << ADJ_ENDSWITCH_PITCH_DOWN)) == 0;
  blCalubrationButtonPushed=(iSensorPinStates & (1 << ADJ_CALIBRATION_BUTTON)) == 0;
  blMotorPower=(iSensorPinStates & (1 << ADJ_MOTOR_POWER)) != 0;
}

bool Multiplexer::EndSwitchRollLeft(){
  return blEndSwitchRollLeft;
}

bool Multiplexer::EndSwitchRollRight(){
  return blEndSwitchRollRight;
}

bool Multiplexer::EndSwitchPitchUp(){
  return blEndSwitchPitchUp;
}

bool Multiplexer::EndSwitchPitchDown(){
  return blEndSwitchPitchDown;
}

bool Multiplexer::CalibrationButtonPushed(){
  return blCalubrationButtonPushed;
}

uint16_t Multiplexer::getYokeButtonPinStates(){
  return iYokeButtonPinStates;
}

uint16_t  Multiplexer::getSensorPinStates(){
  return iSensorPinStates;
}

bool Multiplexer::MotorPower(){
  return blMotorPower;
}

// // Method to update the joystick buttons based on multiplexer input
void Multiplexer::UpdateJoystickButtons() {
// Bit-Shift um 12 für Hat-Switch-Position
  uint16_t hatSwitchState = iYokeButtonPinStates << 12;

  // Setze die Hat-Switch-Position
  switch (hatSwitchState) {
    case 0B0000000000000000:
      joy->setHatSwitch(0, -1); // no direction
      break;
    case 0B0100000000000000:
      joy->setHatSwitch(0, 0); // up
      break;
    case 0B0101000000000000:
      joy->setHatSwitch(0, 45); // up right
      break;
    case 0B0001000000000000:
      joy->setHatSwitch(0, 90); // right
      break;
    case 0B0011000000000000:
      joy->setHatSwitch(0, 135); // down right
      break;
    case 0B0010000000000000:
      joy->setHatSwitch(0, 180); // down
      break;
    case 0B1010000000000000:
      joy->setHatSwitch(0, 225); // down left
      break;
    case 0B1000000000000000:
      joy->setHatSwitch(0, 270); // left
      break;
    case 0B1100000000000000:
      joy->setHatSwitch(0, 315); // up left
      break;
    default:
      break; // no change
  }

  // Lese Button-Zustände vom Multiplexer
  for (byte channel = 4; channel < 16; channel++) {
    joy->setButton(channel - 4, (iYokeButtonPinStates >> channel) & 1);
  }
}
