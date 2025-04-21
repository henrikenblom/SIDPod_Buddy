#include <Arduino.h>
#include <chrono>
#include <set>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include "BuddyInterface.h"
#include "TM0XX0XX.h"

#define VERTICAL_STEP_SIZE   50
#define HORIZONTAL_STEP_SIZE  40
#define ROTATE_STEP_SIZE     15

struct _historyData {
    uint16_t xValue;
    uint16_t yValue;
    double degreeValue;
};

enum Gesture {
    G_NONE = 0,
    G_HORIZONTAL = 1,
    G_VERTICAL = 2,
    G_ROTATE = 3,
    G_TAP = 4,
    G_DOUBLE_TAP = 5,
    G_HOME = 6,
};

uint16_t lastX, lastY = 0;
bool inSession, lastAccumulatedDeltaGTZero, gestureSent = false;
Gesture currentGesture = G_NONE;
std::chrono::milliseconds lastMovementTime, lastStepUpdateTime, lastSessionEndTime;
int touchSamples, touchDowns, currentStepSize = 0;
std::vector<_historyData> historyVector;
std::set<String> btDevices;
double accumulatedDelta, lastDegrees = 0;

absData_t touchData;

AudioInfo info44k1(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
I2SStream i2s;

constexpr int BYTES_PER_FRAME = 4;

int32_t get_sound_data(Frame *data, const int32_t frameCount) {
    return static_cast<int32_t>(i2s.readBytes(reinterpret_cast<uint8_t *>(data), frameCount * BYTES_PER_FRAME)) /
           BYTES_PER_FRAME;
}

bool btDeviceIsValid(const char *ssid, esp_bd_addr_t address, int rssi) {
    btDevices.emplace(ssid);
    return false;
}

void setupPins() {
    pinMode(TAP_PIN, OUTPUT);
    pinMode(VERTICAL_PIN, OUTPUT);
    pinMode(HORIZONTAL_PIN, OUTPUT);
    pinMode(ROTATE_PIN, OUTPUT);
    pinMode(MODIFIER1_PIN, OUTPUT);
    pinMode(MODIFIER2_PIN, OUTPUT);
    pinMode(BT_CONNECTED_PIN, OUTPUT);
}

void setup() {
    setCpuFrequencyMhz(80);
    Serial.begin(115200);
    Serial1.begin(115200);
    setupPins();

    auto cfg = i2s.defaultConfig(RX_MODE);
    cfg.i2s_format = I2S_STD_FORMAT;
    cfg.is_master = false;
    cfg.set(info44k1);
    i2s.begin(cfg);

    Pinnacle_Init();
    a2dp_source.set_ssid_callback(btDeviceIsValid);
    //a2dp_source.start("84-1511", get_sound_data);
    a2dp_source.start();

    Serial.println("Ready");
}

double toDegrees(_absData *absData) {
    const int deltaX = absData->xValue - lastX;
    const int deltaY = absData->yValue - lastY;
    const double rad = atan2(deltaY, deltaX);
    double degrees = rad * 180 / M_PI;
    return degrees;
}

int getVerticalDelta(_absData *absData) {
    return absData->yValue - lastY;
}

int getHorizontalDelta(_absData *absData) {
    return absData->xValue - lastX;
}

double getRotationalDelta(double degrees) {
    double delta = degrees - lastDegrees;
    if (delta < -180) {
        delta += 360;
    } else if (delta > 180) {
        delta -= 360;
    }
    return delta;
}

Gesture identifyGesture(const std::vector<_historyData> &historyVector) {
    auto xSorted = historyVector;
    std::sort(xSorted.begin(), xSorted.end(), [](const _historyData &a, const _historyData &b) {
        return a.xValue < b.xValue;
    });
    int deltaX = xSorted.back().xValue - xSorted.front().xValue;

    auto ySorted = historyVector;
    std::sort(ySorted.begin(), ySorted.end(), [](const _historyData &a, const _historyData &b) {
        return a.yValue < b.yValue;
    });
    int deltaY = ySorted.back().yValue - ySorted.front().yValue;

    auto degreeSorted = historyVector;
    std::sort(degreeSorted.begin(), degreeSorted.end(), [](const _historyData &a, const _historyData &b) {
        return a.degreeValue < b.degreeValue;
    });
    double deltaDegrees = round(degreeSorted.back().degreeValue - degreeSorted.front().degreeValue);

    int degreeLimit = degreeSorted.front().degreeValue < 0 ? 7 : 18;

    if (deltaDegrees >= degreeLimit
        && ySorted.back().yValue >= 170
        && ySorted.back().yValue <= 400
        && deltaY < 80) {
        return G_ROTATE;
    }
    if (std::abs(deltaX) > std::abs(deltaY)) {
        return G_HORIZONTAL;
    }
    return G_VERTICAL;
}

void bang(const uint8_t pin, const bool modifier1, const bool modifier2 = false) {
    digitalWrite(MODIFIER1_PIN, modifier1 ? HIGH : LOW);
    digitalWrite(MODIFIER2_PIN, modifier2 ? HIGH : LOW);
    delay(10);
    digitalWrite(pin, HIGH);
    delay(40);
    digitalWrite(pin, LOW);
    digitalWrite(MODIFIER1_PIN, LOW);
    digitalWrite(MODIFIER2_PIN, LOW);
}

void sendGesture(const Gesture gesture, const bool modifier = false) {
    switch (gesture) {
        case G_TAP:
            bang(TAP_PIN, false);
            break;
        case G_DOUBLE_TAP:
            bang(TAP_PIN, true);
            break;
        case G_HOME:
            bang(TAP_PIN, true, true);
            break;
        case G_VERTICAL:
            bang(VERTICAL_PIN, modifier);
            break;
        case G_HORIZONTAL:
            bang(HORIZONTAL_PIN, modifier);
            break;
        case G_ROTATE:
            bang(ROTATE_PIN, modifier);
            break;
        default:
            break;
    }
    gestureSent = true;
}

int getStepSizeForCurrentGesture() {
    switch (currentGesture) {
        case G_HORIZONTAL:
            return HORIZONTAL_STEP_SIZE;
        case G_VERTICAL:
            return VERTICAL_STEP_SIZE;
        default:
            return ROTATE_STEP_SIZE;
    }
}

void loop() {
    if (millis() - lastSessionEndTime.count() > 100 && DR_Asserted()) {
        Pinnacle_GetAbsolute(&touchData);
        if (!touchData.touchDown) {
            touchDowns++;
        }
        if (touchData.zValue > 20) {
            inSession = true;
            ScaleData(&touchData, 1000, 1000);
            if (historyVector.size() > 4 && currentGesture == G_NONE) {
                currentGesture = identifyGesture(historyVector);
                currentStepSize = getStepSizeForCurrentGesture();
            }
            if ((touchData.xValue || touchData.yValue)
                && (lastX != touchData.xValue || lastY != touchData.yValue)) {
                const double degrees = toDegrees(&touchData);
                if (touchSamples > 6 && currentGesture == G_NONE) {
                    historyVector.push_back({
                        touchData.xValue,
                        touchData.yValue,
                        degrees
                    });
                }
                if (currentGesture == G_HORIZONTAL) {
                    accumulatedDelta += getHorizontalDelta(&touchData);
                } else if (currentGesture == G_VERTICAL) {
                    accumulatedDelta -= getVerticalDelta(&touchData);
                } else if (currentGesture == G_ROTATE) {
                    accumulatedDelta += getRotationalDelta(degrees);
                }
                lastDegrees = degrees;
            }
            lastX = touchData.xValue;
            lastY = touchData.yValue;
            lastMovementTime = std::chrono::milliseconds(millis());
            if (accumulatedDelta > currentStepSize || accumulatedDelta < currentStepSize * -1) {
                lastAccumulatedDeltaGTZero = accumulatedDelta > 0;
                sendGesture(currentGesture, lastAccumulatedDeltaGTZero);
                accumulatedDelta = 0;
                lastStepUpdateTime = std::chrono::milliseconds(millis());
            } else if (currentGesture == G_VERTICAL
                       && millis() - lastStepUpdateTime.count() > 350
                       && (lastY > 667 || lastY < 333)) {
                sendGesture(currentGesture, lastAccumulatedDeltaGTZero);
            }
            touchSamples++;
        }
    } else if (inSession && millis() - lastMovementTime.count() > 180) {
        inSession = false;
        if (touchSamples > 3 && touchSamples < 20 && !gestureSent) {
            if (touchDowns <= 5) {
                sendGesture(G_TAP);
            } else if (lastY < 333) {
                sendGesture(G_HOME);
            } else {
                sendGesture(G_DOUBLE_TAP);
            }
        }
        accumulatedDelta = 0;
        touchSamples = 0;
        touchDowns = 0;
        lastY = 0;
        lastX = 0;
        lastDegrees = 0;
        historyVector.clear();
        currentGesture = G_NONE;
        gestureSent = false;
        lastSessionEndTime = std::chrono::milliseconds(millis());
    }

    if (Serial1.available()) {
        char type = Serial1.read();
        if (type == static_cast<char>(RT_BT_LIST)) {
            Serial1.flush();
            for (const auto &device: btDevices) {
                Serial1.println(device);
            }
        } else if (type == static_cast<char>(RT_BT_SELECT)) {
            Serial.println("MT_BT_SELECT");
            auto selected = Serial1.readStringUntil('\n');
            Serial1.flush();
            Serial.print("Selected device: ");
            Serial.println(selected);
        }
    }
}
