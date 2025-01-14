#include "MotorController.h"

MotorController::MotorController(Multiplexer& multiplexer, byte& roll_speed, 
                byte& pitch_speed) : mux(multiplexer), roll_speed(roll_speed), pitch_speed(pitch_speed) {}

void MotorController::EnableMotors() {
    digitalWrite(ROLL_PITCH_EN, HIGH);
    //digitalWrite(ROLL_EN, HIGH);
}

void MotorController::DisableMotors() {
    digitalWrite(ROLL_PITCH_EN, LOW);
    //digitalWrite(ROLL_EN, LOW);

    analogWrite(ROLL_L_PWM, 0);  // Stop left
    analogWrite(ROLL_R_PWM, 0);  // Stop right
    roll_speed = 0;              // Speed to 0

    analogWrite(PITCH_U_PWM, 0);  // Stop left
    analogWrite(PITCH_D_PWM, 0);  // Stop right
    pitch_speed = 0;              // Speed to 0
}

void MotorController::PrepareMotors(int16_t forces[], int16_t forceMax[], byte pwmMin[], byte pwmMax[]) {
    // Prepare motor for pitch direction
    MoveMotorByForce(pitch_speed,
                     (mux.EndSwitchPitchDown() || mux.EndSwitchPitchUp()),
                     PITCH_U_PWM,
                     PITCH_D_PWM,
                     forces[MEM_PITCH],
                     forceMax[MEM_PITCH],
                     pwmMin[MEM_PITCH],
                     pwmMax[MEM_PITCH]);

    // Prepare motor for roll direction
    MoveMotorByForce(roll_speed,
                     (mux.EndSwitchRollLeft() || mux.EndSwitchRollRight()),
                     ROLL_L_PWM, 
                     ROLL_R_PWM, 
                     forces[MEM_ROLL],
                     forceMax[MEM_ROLL],
                     pwmMin[MEM_ROLL],
                     pwmMax[MEM_ROLL]);
}

void MotorController::MoveMotorByForce(byte &rSpeed, bool blEndSwitch, byte pinLPWM, byte pinRPWM, int16_t gForce, int forceMax, byte pwmMin, byte pwmMax) {
    // If position is on end switch then stop the motor
    if (blEndSwitch) {
        analogWrite(pinLPWM, 0);  // Stop left
        analogWrite(pinRPWM, 0);  // Stop right
        rSpeed = 0;               // Speed to 0
    } else {
        // Cut force to maximum value
        int pForce = constrain(abs(gForce), 0, forceMax);
        // Serial.println(gForce);
        // Serial.println(pForce);
        // Serial.println(forceMax);   
        // Serial.println(pwmMin);
        // Serial.println(pwmMax);
        // Serial.println(rSpeed); 

        // Calculate motor speed (pwm) by force between min pwm and max pwm speed
        rSpeed = map(pForce, 0, forceMax, pwmMin, pwmMax);

        // Which direction?
        if (gForce > 0) {
            analogWrite(pinRPWM, 0);       // Stop right
            analogWrite(pinLPWM, rSpeed);  // Speed up left
        } else {
            analogWrite(pinLPWM, 0);       // Stop left
            analogWrite(pinRPWM, rSpeed);  // Speed up right
        }
    }
}