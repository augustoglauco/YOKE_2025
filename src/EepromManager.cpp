#include "EepromManager.h"
#include <EEPROM.h>
#include "defines.h"

EepromManager::EepromManager() {
    // Constructor implementation (if needed)
}

void EepromManager::clearEEPROM() {
    for (unsigned i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
    }
}

byte EepromManager::isDataAvailable() {
    return EEPROM.read(EEPROM_DATA_AVAILABLE_INDEX);
}

void EepromManager::writeByteArray(int &address, byte values[], byte arraySize) {
    for (int i = 0; i < arraySize; i++) {
        EEPROM.put(address, values[i]);
        address += sizeof(byte);
    }
}

void EepromManager::readByteArray(int &address, byte array[], byte arraySize) {
    for (int i = 0; i < arraySize; i++) {
        EEPROM.get(address, array[i]);
        address += sizeof(byte);
    }
}

void EepromManager::writeInt16Array(int &address, int16_t values[], byte arraySize) {
    for (int i = 0; i < arraySize; i++) {
        EEPROM.put(address, values[i]);
        address += sizeof(int16_t);
    }
}

void EepromManager::readInt16Array(int &address, int16_t array[], byte arraySize) {
    for (int i = 0; i < arraySize; i++) {
        EEPROM.get(address, array[i]);
        address += sizeof(int16_t);
    }
}

void EepromManager::writeInt16(int &address, int16_t value) {
    EEPROM.put(address, value);
    address += sizeof(int16_t);
}

void EepromManager::readInt16(int &address, int16_t &value) {
    EEPROM.get(address, value);
    address += sizeof(int16_t);
}

void EepromManager::writeDataToEEPROM(Gains* gains, EffectParams* effects, int16_t* adjForceMax, byte* adjPwmMin, byte* adjPwmMax) {	
    int eeAddress = EEPROM_DATA_INDEX;

    writeInt16Array(eeAddress, adjForceMax, MEM_AXES);
    writeByteArray(eeAddress, adjPwmMin, MEM_AXES);
    writeByteArray(eeAddress, adjPwmMax, MEM_AXES);

    for (int i = 0; i < MEM_AXES; i++) {
        EEPROM.put(eeAddress, effects[i]);
        eeAddress += sizeof(EffectParams);
    }

    for (int i = 0; i < MEM_AXES; i++) {
        EEPROM.put(eeAddress, gains[i]);
        eeAddress += sizeof(Gains);
    }

    EEPROM.update(EEPROM_DATA_AVAILABLE_INDEX, 1);
}

void EepromManager::readDataFromEEPROM(Gains* gains, EffectParams* effects, int16_t* adjForceMax, byte* adjPwmMin, byte* adjPwmMax) {
    int eeAddress = EEPROM_DATA_INDEX;

    readInt16Array(eeAddress, adjForceMax, MEM_AXES);
    readByteArray(eeAddress, adjPwmMin, MEM_AXES);
    readByteArray(eeAddress, adjPwmMax, MEM_AXES);

    for (int i = 0; i < MEM_AXES; i++) {
        EEPROM.get(eeAddress, effects[i]);
        eeAddress += sizeof(EffectParams);
    }

    for (int i = 0; i < MEM_AXES; i++) {
        EEPROM.get(eeAddress, gains[i]);
        eeAddress += sizeof(Gains);
    }
}