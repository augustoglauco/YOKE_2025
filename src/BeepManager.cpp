#include "BeepManager.h"
#include "defines.h"

// constructor
BeepManager::BeepManager(int pin) {
  buzzerPin = pin;
  pinMode(buzzerPin, OUTPUT);
}

// create tone
void BeepManager::manualTone(int frequency, int duration) {
  long period = 1000000L / frequency;  // Berechne die Periode in Mikrosekunden
  long cycles = (long)duration * 1000L / period;  // Anzahl der Zyklen

  for (long i = 0; i < cycles; i++) {
    digitalWrite(buzzerPin, HIGH);
    delayMicroseconds(period / 2);  // Halbe Periode HIGH
    digitalWrite(buzzerPin, LOW);
    delayMicroseconds(period / 2);  // Halbe Periode LOW
  }
}

// create a beep
void BeepManager::beep(int duration, int frequency) {
  manualTone(frequency, duration); // create beep
  delay(200);                      // delay
}

// // beep code for pitch axis
// void BeepManager::calibrationBeepPitch() {
//   beep(BEEP_LONG_TONE, 1000);  
//   beep(BEEP_SHORT_TONE, 500);  
//   delay(300);
// }

// // beep code for roll axis
// void BeepManager::calibrationBeepRoll() {
//   beep(BEEP_LONG_TONE, 500);  
//   beep(BEEP_SHORT_TONE, 1000);  
//   delay(300);
// }

// Roll or Pitch code?
// void BeepManager::calibrationBeepAxis(bool isRoll) {
//   if(isRoll)
//   {
//     calibrationBeepRoll();
//   }else{
//     calibrationBeepPitch();
//   }
// }

// system start
void BeepManager::systemStart() {
  beep(200, 1000); 
  delay(1000);
}

// start calibration
void BeepManager::calibrationStart() {
  beep(500, 600);  
  delay(500);
  beep(500, 600);  
  delay(500);
  beep(500, 600);  

  delay(1000);
}

// calibration successful finished
void BeepManager::calibrationSuccess() {
  beep(100, 500);  
  delay(100);
  beep(100, 600); 
  delay(100);
  beep(300, 700); 

  delay(1000);
}

// Error on calibration
void BeepManager::calibrationError() {
  beep(100, 600);  
  delay(100);
  beep(100, 600);  
  delay(100);
  beep(100, 600);  
  delay(200);

  beep(300, 600);  
  delay(200);
  beep(300, 600);  
  delay(200);
  beep(300, 600);  
  delay(200);

  beep(100, 600);  
  delay(100);
  beep(100, 600);  
  delay(100);
  beep(100, 600);  

  delay(1000);
}

// axis not moving timeout (short, short)
void BeepManager::calibrationTimeoutMotor() {
  for(byte x=0; x<=BEEP_CODE_COUNT;x++)
  {
    //calibrationBeepAxis(isRoll);
    beep(BEEP_SHORT_TONE, BEEP_CODE_FREQUENCY);  
    beep(BEEP_SHORT_TONE, BEEP_CODE_FREQUENCY);  
    delay(BEEP_CODE_DELAY);
  }
}

// timeout for calibration (short, long)
void BeepManager::calibrationTimeoutGeneral() {
  for(byte x=0; x<=BEEP_CODE_COUNT;x++)
  {
    //calibrationBeepAxis(isRoll);
    beep(BEEP_SHORT_TONE, BEEP_CODE_FREQUENCY);  
    beep(BEEP_LONG_TONE, BEEP_CODE_FREQUENCY);  
    delay(BEEP_CODE_DELAY);
  }
}

// motor goes in wrong direction (long, short)
void BeepManager::calibrationMotorInverted() {
  for(byte x=0; x<=BEEP_CODE_COUNT;x++)
  {
    //calibrationBeepAxis(isRoll);
    beep(BEEP_LONG_TONE, BEEP_CODE_FREQUENCY);  
    beep(BEEP_SHORT_TONE, BEEP_CODE_FREQUENCY);  
    delay(BEEP_CODE_DELAY);
  }
}

// encoder counts in wrong direction (long,long)
void BeepManager::calibrationEncoderInverted() {
  for(byte x=0; x<=BEEP_CODE_COUNT;x++)
  {
    //calibrationBeepAxis(isRoll);
    beep(BEEP_LONG_TONE, BEEP_CODE_FREQUENCY);  
    beep(BEEP_LONG_TONE, BEEP_CODE_FREQUENCY); 
    delay(BEEP_CODE_DELAY);
  }
}

