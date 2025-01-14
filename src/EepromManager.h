#ifndef EEPROMMANAGER_H
#define EEPROMMANAGER_H
#include <Arduino.h>
#include <Joystick.h>

#define EEPROM_DATA_AVAILABLE_INDEX 0     // eeprom address to indicate data available
#define EEPROM_DATA_INDEX 10              // eeprom start address for data
//#define MEM_AXES  2

class EepromManager {
public:
    EepromManager();
    void clearEEPROM();
    byte isDataAvailable();
    void writeDataToEEPROM(Gains* gains, EffectParams* effects, int16_t* adjForceMax, byte* adjPwmMin, byte* adjPwmMax);
    void readDataFromEEPROM(Gains* gains, EffectParams* effects, int16_t* adjForceMax, byte* adjPwmMin, byte* adjPwmMax);

private:
    void writeByteArray(int &address, byte values[], byte arraySize);
    void readByteArray(int &address, byte array[], byte arraySize);
    void writeInt16Array(int &address, int16_t values[], byte arraySize);
    void readInt16Array(int &address, int16_t array[], byte arraySize);
    void writeInt16(int &address, int16_t value);
    void readInt16(int &address, int16_t &value);
};

#endif // EEPROMMANAGER_H