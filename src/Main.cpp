#include <Arduino.h>
#include "defines.h"
#include "EepromManager.h"
#include "Joy.h"
#include "AxisCalibration.h"
#include "MotorController.h"
#include "Communication.h"
#include "BeepManager.h"

bool blCalibration =  true;           // start flag
bool isCalibrationPressed = false;  // calibration flag
unsigned long currentMillis;        // millis for the current loop
unsigned long nextEffectsMillis;    // count millis for next Effect update
unsigned long nextJoystickMillis;   // count millis for next joystick update
const byte MUX_EN_INPUT_ADD1 = B00000010;       // Multiplexer Adjustemts for Calib Button, Force Potis, End Switches
byte roll_speed = 0;
byte pitch_speed = 0;

EepromManager eepromManager;   // Create a EepromManager instance
Joy joy(eepromManager);        // Create a Joy instance and pass the eepromManager object
Multiplexer mux(&joy.getJoystick());  // Create a Multiplexer instance and pass the joy object
JoyData joyData;                      // Joystick data
MotorController motorController(mux, roll_speed, pitch_speed);                      // Create a MotorController instance and pass the mux object
Encoder counterRoll(ROLL2_PIN, ROLL1_PIN);                // init encoder library for roll ir sensor
Encoder counterPitch(PITCH2_PIN, PITCH1_PIN);             // init encoder library for pitch ir sensor
BeepManager beepManager(BUZZER_PIN);  // Instanz der BeepManager Klasse
AxisConfiguration rollConfig = {false, -256, 256, false, false};  // Define rollConfig
AxisConfiguration pitchConfig = {false, -256, 256, false, false}; // Define pitchConfig
Axis rollAxis(ROLL_L_PWM, ROLL_R_PWM, true, &counterRoll, &mux, &beepManager);            // Create a Axis instance for roll and pass the counterRoll and mux object
Axis pitchAxis(PITCH_U_PWM, PITCH_D_PWM, false, &counterPitch, &mux, &beepManager);       // Create a Axis instance for pitch and pass the counter
Communication comm(joy, mux, rollConfig, pitchConfig, counterRoll, counterPitch, roll_speed, pitch_speed);          // Create a Communication instance and initialize serial communication

void ArduinoSetup();    // Arduino setup function
bool CalibrateAxis(Axis& axis, AxisConfiguration& config); // Calibrate axis function

void setup() {  

    nextEffectsMillis = 0;
    nextJoystickMillis=0;

    //Initialize Arduino
    comm.begin(SERIAL_BAUD);                            // Initialize serial communication
    ArduinoSetup();                                     // Initialize Arduino pins            
    joy.initializeJoystick();                           // Initialize Joystick
    joy.SetRangeJoystick(&rollConfig, &pitchConfig);    // Set Joystick range
    motorController.EnableMotors();                     // Enable motors    
    delay(500);
}

void loop() {

    currentMillis = millis(); // number of milliseconds passed since the Arduino board began running the current program
    mux.ReadMux();            // Read values of buttons and end switch sensors (except yoke axes)
 
    if (mux.MotorPower()) {     // check if motor power is on

        if(blCalibration)       // check if calibration is needed
        { 
            Serial.println("Begin Calibration ...");    
            beepManager.calibrationStart(); // Calibration Start
            delay(500);

            if (CalibrateAxis(rollAxis, rollConfig) && CalibrateAxis(pitchAxis, pitchConfig)) { 
                joy.SetRangeJoystick(&rollConfig, &pitchConfig);        
            }

            blCalibration=false;
            Serial.println("End Calibration ..."); 
        } else {
            if(mux.CalibrationButtonPushed())
                isCalibrationPressed = true;
            else if(isCalibrationPressed) {
                    isCalibrationPressed = false;
                    blCalibration=true;            
            }    

            if (currentMillis >= nextJoystickMillis) {                          // do not run more frequently than these many milliseconds, the window system needs time to process
                Joystick_ j = joy.getJoystick();                                // set Joystick buttons
                j.sendState();                                                  // send joystick values to system
            
                if (currentMillis >= nextEffectsMillis) {                       // we calculate condition forces every 100ms or more frequently if we get position updates
                    joy.UpdateEffects(true, &counterRoll, &counterPitch, &rollConfig, &pitchConfig);  // update/calculate new effect paraeters
                    nextEffectsMillis = currentMillis + 100;                    // set time for new effect loop
                } else {
                    joy.UpdateEffects(false, &counterRoll, &counterPitch, &rollConfig, &pitchConfig); // calculate forces without recalculating condition forces
                    //this helps having smoother spring/damper/friction
                    //if our update rate matches our input device
                }   //nextEffectsMillis  || pos_updated

                joyData = joy.getJoyData();             // get joystick data    
               
                motorController.PrepareMotors(*joyData.forces, *joyData.adjForceMax, *joyData.adjPwmMin, *joyData.adjPwmMax); // move motors
                nextJoystickMillis = currentMillis + 20;   // set time for new joystick loop
            }
        }     
    } else{
        beepManager.NoMotorPower();
    }       
    comm.SerialEvent();     // Serial communication event
}

bool CalibrateAxis(Axis& axis, AxisConfiguration& config) { 
    if (axis.Calibrate()) { 
        config = axis.GetConfiguration(); 
        return true; 
    } 
    return false;
}

void ArduinoSetup() {
    // Pitch Pins
    pinMode(ROLL_PITCH_EN, OUTPUT);
    pinMode(PITCH_U_PWM, OUTPUT);
    pinMode(PITCH_D_PWM, OUTPUT);

    // Roll Pins
    pinMode(ROLL_R_PWM, OUTPUT);
    pinMode(ROLL_L_PWM, OUTPUT);

    // Buttons Pins (Multiplexer)
    pinMode(MUX_S0, OUTPUT);
    pinMode(MUX_S1, OUTPUT);
    pinMode(MUX_S2, OUTPUT);
    pinMode(MUX_S3, OUTPUT);

    pinMode(MUX_EN_INPUT, OUTPUT);
    pinMode(MUX_SIGNAL_INPUT, INPUT);

    // Define pin default states
    // Pitch
    digitalWrite(ROLL_PITCH_EN, LOW);
    digitalWrite(PITCH_U_PWM, LOW);
    digitalWrite(PITCH_D_PWM, LOW);
    // Roll
    //digitalWrite(ROLL_EN, LOW);
    digitalWrite(ROLL_R_PWM, LOW);
    digitalWrite(ROLL_L_PWM, LOW);
    // Multiplexer
    digitalWrite(MUX_S0, LOW);
    digitalWrite(MUX_S1, LOW);
    digitalWrite(MUX_S2, LOW);
    digitalWrite(MUX_S3, LOW);

    // Not for all Arduinos!
    // This sets the PWM Speed to maximum for noise reduction

    // Timer1: pins 9 & 10
    TCCR1B = _BV(CS10);  // Change the PWM frequency to 31.25kHz - pins 9 & 10

    // Timer4: pin 13 & 6
    TCCR4B = _BV(CS40);  // Change the PWM frequency to 31.25kHz - pin 13 & 6
}