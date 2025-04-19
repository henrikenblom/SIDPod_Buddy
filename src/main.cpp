#include <Arduino.h>
#include <chrono>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include <SPI.h>

// ___ Using a Cirque TM040040 with an Arduino ___
// This demonstration application is built to work with a Teensy 3.1/3.2 but it can easily be adapted to
// work with Arduino-based systems.
// This application connects to a TM040040 circular touch pad via SPI. To verify that your touch pad is configured
// for SPI-mode, make sure that R1 is populated with a 470k resistor (or whichever resistor connects pins 24 & 25 of the 1CA027 IC).
// The pad is configured for Absolute mode tracking.  Touch data is sent in text format over USB CDC to
// the host PC.  You can open a terminal window on the PC to the USB CDC port and see X, Y, and Z data
// scroll up the window when you touch the sensor. Tools->Serial Monitor can be used to view touch data.

//  Pinnacle TM040040 with Arduino
//  Hardware Interface
//  GND
//  +3.3V
//  SCK = Pin 13
//  MISO = Pin 12
//  MOSI = Pin 11
//  SS = PIN 10
//  DR = Pin 9

// Hardware pin-number labels
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

typedef struct _relData {
    int16_t xDelta;
    int16_t yDelta;
    uint8_t buttonFlags;
    int8_t scrollWheel;
} relData_t;

enum Movement {
    HORIZONTAL,
    VERTICAL,
    ROTATE,
    CLICK
};

struct _historyData {
    uint16_t xValue;
    uint16_t yValue;
    double degreeValue;
};

uint16_t lastX = 0;
uint16_t lastY = 0;
double lastDegrees = 0;
bool inSession = false;
std::string currentMovement = "None";
std::chrono::milliseconds lastMovementTime;
int touchSamples = 0;
std::vector<_historyData> historyVector;

relData_t relData;

absData_t touchData;

AudioInfo info44k1(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
I2SStream i2s;


constexpr int BYTES_PER_FRAME = 4;

int32_t get_sound_data(Frame *data, const int32_t frameCount) {
    return static_cast<int32_t>(i2s.readBytes(reinterpret_cast<uint8_t *>(data), frameCount * BYTES_PER_FRAME)) /
           BYTES_PER_FRAME;
}

void avoidWatchdogReboots() {
    delay(1000);
}

/*  Pinnacle-based TM040040 Functions  */


/*  I/O Functions */
void Assert_CS() {
    digitalWrite(CS_PIN, LOW);
}

void DeAssert_CS() {
    digitalWrite(CS_PIN, HIGH);
}

void AssertSensorLED(bool state) {
    //digitalWrite(LED_0, !state);
}

bool DR_Asserted() {
    return digitalRead(DR_PIN);
}

/*  RAP Functions */
// Reads <count> Pinnacle registers starting at <address>
void RAP_ReadBytes(uint8_t address, uint8_t *data, uint8_t count) {
    uint8_t cmdByte = READ_MASK | address; // Form the READ command byte

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    Assert_CS();
    SPI.transfer(cmdByte); // Signal a RAP-read operation starting at <address>
    SPI.transfer(0xFC); // Filler byte
    SPI.transfer(0xFC); // Filler byte
    for (uint8_t i = 0; i < count; i++) {
        data[i] = SPI.transfer(0xFC); // Each subsequent SPI transfer gets another register's contents
    }
    DeAssert_CS();

    SPI.endTransaction();
}


// Writes single-byte <data> to <address>
void RAP_Write(uint8_t address, uint8_t data) {
    uint8_t cmdByte = WRITE_MASK | address; // Form the WRITE command byte

    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

    Assert_CS();
    SPI.transfer(cmdByte); // Signal a write to register at <address>
    SPI.transfer(data); // Send <value> to be written to register
    DeAssert_CS();

    SPI.endTransaction();
}

// Clears Status1 register flags (SW_CC and SW_DR)
void Pinnacle_ClearFlags() {
    RAP_Write(0x02, 0x00);
    delayMicroseconds(50);
}

// Enables/Disables the feed
void Pinnacle_EnableFeed(bool feedEnable) {
    uint8_t temp;

    RAP_ReadBytes(0x04, &temp, 1); // Store contents of FeedConfig1 register

    if (feedEnable) {
        temp |= 0x01; // Set Feed Enable bit
        RAP_Write(0x04, temp);
    } else {
        temp &= ~0x01; // Clear Feed Enable bit
        RAP_Write(0x04, temp);
    }
}


void Pinnacle_Init() {
    DeAssert_CS();
    pinMode(CS_PIN, OUTPUT);
    pinMode(DR_PIN, INPUT);

    SPI.begin();

    // Host clears SW_CC flag
    Pinnacle_ClearFlags();

    // Host configures bits of registers 0x03 and 0x05
    RAP_Write(0x03, SYSCONFIG_1);
    RAP_Write(0x05, FEEDCONFIG_2);

    // Host enables preferred output mode (absolute)
    RAP_Write(0x04, FEEDCONFIG_1);

    // Host sets z-idle packet count to 5 (default is 30)
    RAP_Write(0x0A, Z_IDLE_COUNT);
}

// Reads XYZ data from Pinnacle registers 0x14 through 0x17
// Stores result in absData_t struct with xValue, yValue, and zValue members
void Pinnacle_GetAbsolute(absData_t *result) {
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    RAP_ReadBytes(PACKETBYTE_0_ADDRESS, data, 6);

    Pinnacle_ClearFlags();

    result->buttonFlags = data[0] & 0x3F;
    result->xValue = data[2] | ((data[4] & 0x0F) << 8);
    result->yValue = data[3] | ((data[4] & 0xF0) << 4);
    result->zValue = data[5] & 0x3F;

    result->touchDown = result->xValue != 0;
}

void Pinnacle_GetRelative(relData_t *result) {
    uint8_t data[4] = {0, 0, 0, 0};
    RAP_ReadBytes(PACKETBYTE_0_ADDRESS, data, 4);

    Pinnacle_ClearFlags();

    result->buttonFlags = data[0] & 0x07;
    result->xDelta = data[1] | ((data[0] & 0x10) << 8);
    result->yDelta = data[2] | ((data[0] & 0x20) << 8);
    result->scrollWheel = data[3];
}


/*  ERA (Extended Register Access) Functions  */
// Reads <count> bytes from an extended register at <address> (16-bit address),
// stores values in <*data>
void ERA_ReadBytes(uint16_t address, uint8_t *data, uint16_t count) {
    uint8_t ERAControlValue = 0xFF;

    Pinnacle_EnableFeed(false); // Disable feed

    RAP_Write(0x1C, (uint8_t) (address >> 8)); // Send upper byte of ERA address
    RAP_Write(0x1D, (uint8_t) (address & 0x00FF)); // Send lower byte of ERA address

    for (uint16_t i = 0; i < count; i++) {
        RAP_Write(0x1E, 0x05); // Signal ERA-read (auto-increment) to Pinnacle

        // Wait for status register 0x1E to clear
        do {
            RAP_ReadBytes(0x1E, &ERAControlValue, 1);
        } while (ERAControlValue != 0x00);

        RAP_ReadBytes(0x1B, data + i, 1);

        Pinnacle_ClearFlags();
    }
}

// Writes a byte, <data>, to an extended register at <address> (16-bit address)
void ERA_WriteByte(uint16_t address, uint8_t data) {
    uint8_t ERAControlValue = 0xFF;

    Pinnacle_EnableFeed(false); // Disable feed

    RAP_Write(0x1B, data); // Send data byte to be written

    RAP_Write(0x1C, (uint8_t) (address >> 8)); // Upper byte of ERA address
    RAP_Write(0x1D, (uint8_t) (address & 0x00FF)); // Lower byte of ERA address

    RAP_Write(0x1E, 0x02); // Signal an ERA-write to Pinnacle

    // Wait for status register 0x1E to clear
    do {
        RAP_ReadBytes(0x1E, &ERAControlValue, 1);
    } while (ERAControlValue != 0x00);

    Pinnacle_ClearFlags();
}


/*  Logical Scaling Functions */
// Clips raw coordinates to "reachable" window of sensor
// NOTE: values outside this window can only appear as a result of noise
void ClipCoordinates(absData_t *coordinates) {
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
void ScaleData(absData_t *coordinates, uint16_t xResolution, uint16_t yResolution) {
    uint32_t xTemp = 0;
    uint32_t yTemp = 0;

    ClipCoordinates(coordinates);

    xTemp = coordinates->xValue;
    yTemp = coordinates->yValue;

    // translate coordinates to (0, 0) reference by subtracting edge-offset
    xTemp -= PINNACLE_X_LOWER;
    yTemp -= PINNACLE_Y_LOWER;

    // scale coordinates to (xResolution, yResolution) range
    coordinates->xValue = (uint16_t) (xTemp * xResolution / PINNACLE_X_RANGE);
    coordinates->yValue = (uint16_t) (yTemp * yResolution / PINNACLE_Y_RANGE);
}


void setup() {
    setCpuFrequencyMhz(80);
    Serial.begin(115200);
    auto cfg = i2s.defaultConfig(RX_MODE);
    cfg.i2s_format = I2S_STD_FORMAT;
    cfg.is_master = false;
    cfg.set(info44k1);
    i2s.begin(cfg);

    Pinnacle_Init();
    a2dp_source.start("84-1511", get_sound_data);
    Serial.println("Ready");
}

void printRelData(_relData *relData) {
    Serial.print("X: ");
    Serial.print(relData->xDelta);
    Serial.print(" Y: ");
    Serial.print(relData->yDelta);
    Serial.print(" Scroll: ");
    Serial.print(relData->scrollWheel);
    Serial.print(" Button: ");
    Serial.println(relData->buttonFlags);
}

void printAbsData(_absData *absData) {
    Serial.print("X: ");
    Serial.print(absData->xValue);
    Serial.print(" Y: ");
    Serial.print(absData->yValue);
    Serial.print(" Z: ");
    Serial.print(absData->zValue);
    Serial.print(" Button: ");
    Serial.println(absData->buttonFlags);
}

double toDegrees(_absData *absData) {
    const int deltaX = absData->xValue - lastX;
    const int deltaY = absData->yValue - lastY;
    const double rad = atan2(deltaY, deltaX);
    double degrees = rad * 180 / M_PI;
    // if (degrees < 0) {
    //     degrees += 360;
    // }
    return degrees;
}

int getVerticalChange(_absData *absData) {
    return absData->yValue - lastY;
}

int getHorizontalChange(_absData *absData) {
    return absData->xValue - lastX;
}

// Add this function to identify gestures based on historyVector
Movement identifyGesture(const std::vector<_historyData> &historyVector) {
    // Sort by xValue to calculate horizontal movement
    auto xSorted = historyVector;
    std::sort(xSorted.begin(), xSorted.end(), [](const _historyData &a, const _historyData &b) {
        return a.xValue < b.xValue;
    });
    int deltaX = xSorted.back().xValue - xSorted.front().xValue;

    // Sort by yValue to calculate vertical movement
    auto ySorted = historyVector;
    std::sort(ySorted.begin(), ySorted.end(), [](const _historyData &a, const _historyData &b) {
        return a.yValue < b.yValue;
    });
    int deltaY = ySorted.back().yValue - ySorted.front().yValue;

    // Sort by degreeValue to calculate rotation
    auto degreeSorted = historyVector;
    std::sort(degreeSorted.begin(), degreeSorted.end(), [](const _historyData &a, const _historyData &b) {
        return a.degreeValue < b.degreeValue;
    });
    double deltaDegrees = degreeSorted.back().degreeValue - degreeSorted.front().degreeValue;

    int degreeLimit = 18;
    int deltaXLimit = 200;
    if (degreeSorted.front().degreeValue < 0) {
        degreeLimit = 8;
        deltaXLimit = 300;
    }

    Serial.print("Delta degrees: ");
    Serial.println(deltaDegrees);
    Serial.print("Delta x: ");
    Serial.println(deltaX);
    Serial.print("Delta y: ");
    Serial.println(deltaY);
    Serial.print("Degree limit: ");
    Serial.println(degreeLimit);
    Serial.print("Delta X limit: ");
    Serial.println(deltaXLimit);


    // Determine the gesture
    if (static_cast<int>(deltaDegrees) >= degreeLimit
        && ySorted.back().yValue >= 170
        && ySorted.back().yValue <= 400
        && deltaY < 80) {
        Serial.println("Rotate");
        return ROTATE;
    } else if (std::abs(deltaX) > std::abs(deltaY)) {
        Serial.println("Horizontal");
        return HORIZONTAL;
    } else {
        Serial.println("Vertical");
        return VERTICAL;
    }
}

std::string getMovement() {
    for (_historyData data: historyVector) {
        Serial.print("X: ");
        Serial.print(data.xValue);
        Serial.print(" Y: ");
        Serial.print(data.yValue);
        Serial.print(" Degrees: ");
        Serial.println(data.degreeValue);
    }
    std::sort(historyVector.begin(), historyVector.end(), [](const _historyData &a, const _historyData &b) {
        return a.xValue < b.xValue;
    });
    int deltaX = historyVector.back().xValue - historyVector.front().xValue;

    std::sort(historyVector.begin(), historyVector.end(), [](const _historyData &a, const _historyData &b) {
        return a.yValue < b.yValue;
    });
    int deltaY = historyVector.back().yValue - historyVector.front().yValue;

    std::sort(historyVector.begin(), historyVector.end(), [](const _historyData &a, const _historyData &b) {
        return a.degreeValue < b.degreeValue;
    });
    double deltaDegrees = historyVector.back().degreeValue - historyVector.front().degreeValue;
    double deltaXYRatio = deltaX > deltaY
                              ? static_cast<double>(deltaX) / static_cast<double>(deltaY)
                              : static_cast<double>(deltaY) / static_cast<double>(deltaX);
    if (deltaXYRatio < 0) {
        deltaXYRatio *= -1;
    }
    // Serial.print("deltaDegrees: ");
    // Serial.print(deltaDegrees);
    // Serial.print(" deltaX: ");
    // Serial.print(deltaX);
    // Serial.print(" deltaY: ");
    // Serial.print(deltaY);
    // Serial.print(" deltaDeltaXY: ");
    // Serial.println(deltaXYRatio);
    return std::string("");
}

void loop() {
    if (DR_Asserted()) {
        Pinnacle_GetAbsolute(&touchData);
        if (touchData.xValue > 128 && touchData.xValue < 1920
            && touchData.yValue > 64 && touchData.yValue < 1472
            && touchData.zValue > 20) {
            inSession = true;
            ScaleData(&touchData, 1000, 1000);
            if (historyVector.size() > 6 && currentMovement == "None") {
                currentMovement = getMovement();
                identifyGesture(historyVector);
            }

            if ((touchData.xValue || touchData.yValue)
                && (lastX != touchData.xValue || lastY != touchData.yValue)) {
                double degrees = toDegrees(&touchData);
                if (touchSamples > 6) {
                    historyVector.push_back({
                        touchData.xValue,
                        touchData.yValue,
                        degrees
                    });
                }

                if (currentMovement != "None") {
                    // if (currentMovement == "HORIZONTAL") {
                    //     Serial.print("Horizontal: ");
                    //     Serial.println(getHorizontalChange(&touchData));
                    // } else if (currentMovement == "VERTICAL") {
                    //     Serial.print("Vertical: ");
                    //     Serial.println(getVerticalChange(&touchData));
                    // } else if (currentMovement == "ROTATE") {
                    //Serial.print("Rotate: ");
                    //Serial.println(degrees);
                    // } else if (currentMovement == "CLICK") {
                    //     Serial.println("Click");
                    // } else {
                    //     Serial.println("None");
                    // }
                }
                lastDegrees = degrees;
            }

            lastX = touchData.xValue;
            lastY = touchData.yValue;
            lastMovementTime = std::chrono::milliseconds(millis());
            touchSamples++;
        }
    } else if (inSession && millis() - lastMovementTime.count() > 150) {
        inSession = false;
        if (currentMovement == "None") {
            Serial.println("CLICK");
        }
        Serial.print("Touch samples: ");
        Serial.println(touchSamples);
        touchSamples = 0;
        lastY = 0;
        lastX = 0;
        lastDegrees = 0;
        historyVector.clear();
        currentMovement = "None";
    }
}
