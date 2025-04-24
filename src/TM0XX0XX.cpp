#include <Arduino.h>
#include <SPI.h>
#include "TM0XX0XX.h"

// Hardware pin-number labels

/*  Pinnacle-based TM040040 Functions  */

/*  I/O Functions */
void assertCS() {
    digitalWrite(CS_PIN, LOW);
}

void deAssertCS() {
    digitalWrite(CS_PIN, HIGH);
}

bool drAsserted() {
    return digitalRead(DR_PIN);
}

/*  RAP Functions */
// Reads <count> Pinnacle registers starting at <address>
void rapReadBytes(const uint8_t address, uint8_t *data, const uint8_t count) {
    const uint8_t cmdByte = READ_MASK | address; // Form the READ command byte

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    assertCS();
    SPI.transfer(cmdByte); // Signal a RAP-read operation starting at <address>
    SPI.transfer(0xFC); // Filler byte
    SPI.transfer(0xFC); // Filler byte
    for (uint8_t i = 0; i < count; i++) {
        data[i] = SPI.transfer(0xFC); // Each subsequent SPI transfer gets another register's contents
    }
    deAssertCS();

    SPI.endTransaction();
}


// Writes single-byte <data> to <address>
void rapWrite(const uint8_t address, const uint8_t data) {
    const uint8_t cmdByte = WRITE_MASK | address; // Form the WRITE command byte

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    assertCS();
    SPI.transfer(cmdByte); // Signal a write to register at <address>
    SPI.transfer(data); // Send <value> to be written to register
    deAssertCS();

    SPI.endTransaction();
}

// Clears Status1 register flags (SW_CC and SW_DR)
void pinnacleClearFlags() {
    rapWrite(0x02, 0x00);
    delayMicroseconds(50);
}

// Enables/Disables the feed
void pinnacleEnableFeed(const bool feedEnable) {
    uint8_t temp;

    rapReadBytes(0x04, &temp, 1); // Store contents of FeedConfig1 register

    if (feedEnable) {
        temp |= 0x01; // Set Feed Enable bit
        rapWrite(0x04, temp);
    } else {
        temp &= ~0x01; // Clear Feed Enable bit
        rapWrite(0x04, temp);
    }
}


void pinnacleInit() {
    deAssertCS();
    pinMode(CS_PIN, OUTPUT);
    pinMode(DR_PIN, INPUT);

    SPI.begin();

    // Host clears SW_CC flag
    pinnacleClearFlags();

    // Host configures bits of registers 0x03 and 0x05
    rapWrite(0x03, SYSCONFIG_1);
    rapWrite(0x05, FEEDCONFIG_2);

    // Host enables preferred output mode (absolute)
    rapWrite(0x04, FEEDCONFIG_1);

    // Host sets z-idle packet count to 5 (default is 30)
    rapWrite(0x0A, Z_IDLE_COUNT);
}

// Reads XYZ data from Pinnacle registers 0x14 through 0x17
// Stores result in absData_t struct with xValue, yValue, and zValue members
void pinnacleGetAbsolute(absData_t *result) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    rapReadBytes(PACKETBYTE_0_ADDRESS, data, 6);

    pinnacleClearFlags();

    result->buttonFlags = data[0] & 0x3F;
    result->xValue = data[2] | ((data[4] & 0x0F) << 8);
    result->yValue = data[3] | ((data[4] & 0xF0) << 4);
    result->zValue = data[5] & 0x3F;

    result->touchDown = result->xValue != 0;
}


/*  ERA (Extended Register Access) Functions  */
// Reads <count> bytes from an extended register at <address> (16-bit address),
// stores values in <*data>
void eraReadBytes(const uint16_t address, uint8_t *data, const uint16_t count) {
    uint8_t ERAControlValue = 0xFF;

    pinnacleEnableFeed(false); // Disable feed

    rapWrite(0x1C, static_cast<uint8_t>(address >> 8)); // Send upper byte of ERA address
    rapWrite(0x1D, static_cast<uint8_t>(address & 0x00FF)); // Send lower byte of ERA address

    for (uint16_t i = 0; i < count; i++) {
        rapWrite(0x1E, 0x05); // Signal ERA-read (auto-increment) to Pinnacle

        // Wait for status register 0x1E to clear
        do {
            rapReadBytes(0x1E, &ERAControlValue, 1);
        } while (ERAControlValue != 0x00);

        rapReadBytes(0x1B, data + i, 1);

        pinnacleClearFlags();
    }
}

// Writes a byte, <data>, to an extended register at <address> (16-bit address)
void eraWriteByte(const uint16_t address, const uint8_t data) {
    uint8_t ERAControlValue = 0xFF;

    pinnacleEnableFeed(false); // Disable feed

    rapWrite(0x1B, data); // Send data byte to be written

    rapWrite(0x1C, static_cast<uint8_t>(address >> 8)); // Upper byte of ERA address
    rapWrite(0x1D, static_cast<uint8_t>(address & 0x00FF)); // Lower byte of ERA address

    rapWrite(0x1E, 0x02); // Signal an ERA-write to Pinnacle

    // Wait for status register 0x1E to clear
    do {
        rapReadBytes(0x1E, &ERAControlValue, 1);
    } while (ERAControlValue != 0x00);

    pinnacleClearFlags();
}


/*  Logical Scaling Functions */
// Clips raw coordinates to "reachable" window of sensor
// NOTE: values outside this window can only appear as a result of noise
void clipCoordinates(absData_t *coordinates) {
    if (coordinates->xValue < PINNACLE_X_LOWER) {
        coordinates->xValue = PINNACLE_X_LOWER;
    } else if (coordinates->xValue > PINNACLE_X_UPPER) {
        coordinates->xValue = PINNACLE_X_UPPER;
    }
    if (coordinates->yValue < PINNACLE_Y_LOWER) {
        coordinates->yValue = PINNACLE_Y_LOWER;
    } else if (coordinates->yValue > PINNACLE_Y_UPPER) {
        coordinates->yValue = PINNACLE_Y_UPPER;
    }
}

// Scales data to desired X & Y resolution
void scaleData(absData_t *coordinates, const uint16_t xResolution, const uint16_t yResolution) {
    uint32_t xTemp = 0;
    uint32_t yTemp = 0;

    clipCoordinates(coordinates);

    xTemp = coordinates->xValue;
    yTemp = coordinates->yValue;

    // translate coordinates to (0, 0) reference by subtracting edge-offset
    xTemp -= PINNACLE_X_LOWER;
    yTemp -= PINNACLE_Y_LOWER;

    // scale coordinates to (xResolution, yResolution) range
    coordinates->xValue = static_cast<uint16_t>(xTemp * xResolution / PINNACLE_X_RANGE);
    coordinates->yValue = static_cast<uint16_t>(yTemp * yResolution / PINNACLE_Y_RANGE);
}
