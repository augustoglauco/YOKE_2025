#pragma once
#ifndef AXIS_CALIBRATION_H
    #define AXIS_CALIBRATION_H
    #include <Arduino.h>
    #include <Encoder.h>
    #include "Multiplexer.h"
    #include "BeepManager.h"

    /******************************************
         Calibration Constants
    *******************************************/
    #define CALIBRATION_MAX_SPEED 75;                        // Maximum speed
    #define CALIBRATION_AXIS_MOVEMENT_TIMEOUT 4000;           // Timeout of 4 seconds for no movement
    #define CALIBRATION_TIMEOUT 20000;                        // Timeout of 20 seconds for calibration
    #define CALIBRATION_SPEED_INCREMENT 5;                   // the speed is increased until asix movement, this is added to speed then move´ment indicates 
    #define CALIBRATION_WHILE_DELAY 20;                        // waitdelay inside while of movement to give Arduino time. Change will change speed!
    #define CALIBRATION_WHILE_DELAY_MOTOR_STOPS 30;           // waitdelay when motor stops to give him time to stops
    #define CALIBRATION_DELAY_MOVE_OUT_OF_ENDSTOP 100;        // If asix is on endstop on start od´f calibration it will move out of and wait shot before continue

    //Structure to hold the configuration of the axis
    typedef struct {
        bool blError;              // true if a timeout occurred
        int16_t iMin;              // Minimum value
        int16_t iMax;              // Maximum value
        bool blMotorInverted;      // true if the motor direction is inverted
        bool blEncoderInverted;    // true if the encoder counting direction is inverted
        bool blAxisTimeout;
        bool blTimeout;
    } AxisConfiguration;

    class Axis {
        private:
            Multiplexer* multiplexer;           // Pointer to a multiplexer object
            Encoder* encoder;                   // Pointer to the encoder object
            byte motorPinLeft;                  // Pin for left motor control
            byte motorPinRight;                 // Pin for right motor control
            bool blEndSwitchLeft;               // Pointer to left end switch status
            bool blEndSwitchRight;              // Pointer to right end switch status
            bool blIsRoll;                      // is Roll or Pitch                   
            byte speed;                         // Current speed of the motor
            unsigned long lastMovementTime;     // Timestamp of the last movement
            BeepManager* beepManager;           // Pointer to the BeepManager object


            // Configuration constants
            static const byte maxSpeed = CALIBRATION_MAX_SPEED;                                     // Maximum speed
            static const unsigned long timeout = CALIBRATION_AXIS_MOVEMENT_TIMEOUT;                 // Timeout of 4 seconds for no movement
            static const unsigned long calibrationTimeout = CALIBRATION_TIMEOUT;                    // Timeout of 20 seconds for calibration
            static const byte speedIncrement = CALIBRATION_SPEED_INCREMENT;                         // Speed increment value
            static const byte whileDelay = CALIBRATION_WHILE_DELAY;                                 // delay inside while to give Arduino time to work 
            static const byte waitDelayMotorStops =CALIBRATION_WHILE_DELAY_MOTOR_STOPS;             // delay inside while to give Arduino time to work 
            static const byte waitDelayAfterMoveOutEndstop =CALIBRATION_DELAY_MOVE_OUT_OF_ENDSTOP;  // not too fast

            // Calibration start time
            unsigned long calibrationStartTime; 

            // Axis configuration object
            AxisConfiguration config;

            // Helper method to manage motor movement
            void ManageMovement(bool direction, unsigned long& lastMovementTime, int& lastEncoderValue, bool& speedIncreased);
            void ReadMultiplexer();
            int ResetEncoder();
            bool CheckTimeouts(unsigned long lastMovementTime, unsigned long calibrationStartTime);
            bool ErrorCheck();      


        public:
            // Constructor to initialize motor pins and end switches
            //Axis(int motorLeftPin, int motorRightPin, bool* endSwitchLeft, bool* endSwitchRight, Encoder* encoder, Multiplexer* multiplexerPtr);
            Axis(int motorLeftPin, int motorRightPin, bool isRoll, Encoder* encoder, Multiplexer* multiplexerPtr, BeepManager* beepManager);
                        
            void MoveMotor(bool direction); // Method to move the motor in a given direction            
            void StopMotor(); // Method to stop the motor

            // Calibration method for the axis
            bool Calibrate(); // return true on success, false if errors exists

            // Method to get the current axis configuration
            AxisConfiguration GetConfiguration();
    };

#endif
