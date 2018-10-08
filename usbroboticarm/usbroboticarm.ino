/*
    Modbus slave example.
    
    * Control digital pins mode using holding registers 0 .. 13.
    * Controls digital output pins as modbus coils.
    * Reads digital inputs state as discreet inputs.
    * Reads analog inputs as input registers.
    * Write and Read EEPROM as holding registers.

    Created 8 12 2015
    By Yaacov Zamir

    https://github.com/yaacov/ArduinoModbusSlave

*/

#include <ModbusSlave.h>

/* slave id = 1, control-pin = 8, baud = 9600
 */
#define SLAVE_ID 1
#define CTRL_PIN 99
#define BAUDRATE 115200

uint16_t modbusreg[50];

/**
 *  Modbus object declaration.
 */
Modbus slave(SLAVE_ID, CTRL_PIN);

void setup() {
    
    /* register handler functions.
     * into the modbus slave callback vector.
     */
    slave.cbVector[CB_READ_COILS] = readDigitalIn;
    slave.cbVector[CB_WRITE_COILS] = writeDigitalOut;
    slave.cbVector[CB_READ_REGISTERS] = readMemory;
    slave.cbVector[CB_WRITE_REGISTERS] = writeMemory;
    
    // set Serial and slave at baud 9600.
    Serial.begin( BAUDRATE );
    slave.begin( BAUDRATE );
}

void loop() {
    /* listen for modbus commands con serial port.
     *
     * on a request, handle the request.
     * if the request has a user handler function registered in cbVector.
     * call the user handler function.
     */ 
    slave.poll();
}

/**
 * Handel Read Input Status (FC=02)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code.
 *      uint16_t address - first register/coil address.
 *      uint16_t length/status - length of data / coil status.
 */
uint8_t readDigitalIn(uint8_t fc, uint16_t address, uint16_t length) {
    // check the function code.
    if (fc == FC_READ_COILS) {
        // read coils.
        readCoils(address, length);
        return STATUS_OK;
    }
    
    // read digital input
    for (int i = 0; i < length; i++) {
        // write one boolean (1 bit) to the response buffer.
        slave.writeCoilToBuffer(i, digitalRead(address + i));
    }

    return STATUS_OK;
}

/**
 * Handel Read Coils (FC=01)
 * write back the values from digital in pins (input status).
 */
uint8_t readCoils(uint16_t address, uint16_t length) {
    // read coils state
    for (int i = 0; i < length; i++) {
        // write one boolean (1 bit) to the response buffer.
        slave.writeCoilToBuffer(i, digitalRead(address + i));
    }

    return STATUS_OK;
}

/**
 * Handel Read Holding Registers (FC=03)
 * write back the values from eeprom (holding registers).
 */
uint8_t readMemory(uint8_t fc, uint16_t address, uint16_t length) {
    uint16_t value;
    
    // check the function code
    if (fc == FC_READ_INPUT_REGISTERS) {
        // read eeprom memory
        readAnalogIn(address, length);
        return STATUS_OK;
    }
    
    // read program memory.
    for (int i = 0; i < length; i++) {
        value = modbusreg[address + i];
        
        // write uint16_t value to the response buffer.
        slave.writeRegisterToBuffer(i, value);
    }

    return STATUS_OK;
}

/**
 * Handel Read Input Registers (FC=04)
 * write back the values from analog in pins (input registers).
 */
uint8_t readAnalogIn(uint16_t address, uint16_t length) {
    // read analog input
    for (int i = 0; i < length; i++) {
        // write uint16_t value to the response buffer.
        slave.writeRegisterToBuffer(i, analogRead(address + i));
    }
}

/**
 * Handle Force Single Coil (FC=05) and Force Multiple Coils (FC=15)
 * set digital output pins (coils).
 */
uint8_t writeDigitalOut(uint8_t fc, uint16_t address, uint16_t length) {
    // set digital pin state(s).
    for (int i = 0; i < length; i++) {
        digitalWrite(address + i, slave.readCoilFromBuffer(i));
    }

    return STATUS_OK;
}

/**
 * Handle Write Holding Register(s) (FC=06, FC=16)
 * write data into eeprom.
 */
uint8_t writeMemory(uint8_t fc, uint16_t address, uint16_t length) {
    uint16_t value;
    uint16_t registerIndex;
    
    // write to eeprom.
    for (int i = 0; i < length; i++) {
        registerIndex = address + i;
        
        // get uint16_t value from the request buffer.
        value = slave.readRegisterFromBuffer(i);
        
        modbusreg[registerIndex] = value;
    }

    return STATUS_OK;
}
