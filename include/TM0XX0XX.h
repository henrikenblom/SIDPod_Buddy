//
// Created by Henrik Enblom on 2025-04-21.
//

#ifndef TM0XX0XX_H
#define TM0XX0XX_H

#define SCK_PIN   SCK
#define DIN_PIN   MISO
#define DOUT_PIN  MOSI
#define CS_PIN    SS
#define DR_PIN    17

// Masks for Cirque Register Access Protocol (RAP)
#define WRITE_MASK  0x80
#define READ_MASK   0xA0

// Register config values for this demo
#define SYSCONFIG_1   0x00
#define FEEDCONFIG_1  0x03
#define FEEDCONFIG_2  0x1F
#define Z_IDLE_COUNT  0x05

#define PACKETBYTE_0_ADDRESS 0x12


// Coordinate scaling values
#define PINNACLE_XMAX     2047    // max value Pinnacle can report for X
#define PINNACLE_YMAX     1535    // max value Pinnacle can report for Y
#define PINNACLE_X_LOWER  127     // min "reachable" X value
#define PINNACLE_X_UPPER  1919    // max "reachable" X value
#define PINNACLE_Y_LOWER  63      // min "reachable" Y value
#define PINNACLE_Y_UPPER  1471    // max "reachable" Y value
#define PINNACLE_X_RANGE  (PINNACLE_X_UPPER-PINNACLE_X_LOWER)
#define PINNACLE_Y_RANGE  (PINNACLE_Y_UPPER-PINNACLE_Y_LOWER)

// Convenient way to store and access measurements
typedef struct _absData {
    uint16_t xValue;
    uint16_t yValue;
    uint16_t zValue;
    uint8_t buttonFlags;
    bool touchDown;
} absData_t;

void pinnacleInit();

bool drAsserted();

void pinnacleGetAbsolute(absData_t *result);

void scaleData(absData_t *coordinates, uint16_t xResolution, uint16_t yResolution);

#endif //TM0XX0XX_H
