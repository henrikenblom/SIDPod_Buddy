#include <Arduino.h>
#include <chrono>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include "BuddyInterface.h"
#include "TM0XX0XX.h"
#include "main.h"

#include <map>

uint16_t lastX, lastY = 0;
bool inSession, lastAccumulatedDeltaGTZero, gestureSent = false;
Gesture currentGesture, forcedGesture = G_NONE;
std::chrono::milliseconds lastMovementTime, lastStepUpdateTime, lastSessionEndTime, lastDeviceValidation;
int touchSamples, touchDowns, currentStepSize, connectionAttempts = 0;
std::vector<_historyData> historyVector;
std::map<std::string, std::array<uint8_t, 6> > btDevices;
double accumulatedDelta, lastDegrees = 0;
std::string selectedSSID;
esp_bd_addr_t selectedAddress;

absData_t touchData;

AudioInfo info44k1(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
I2SStream i2s;

constexpr int BYTES_PER_FRAME = 4;

void setConnected(bool connected) {
    Serial.print("Connected: ");
    Serial.println(connected);
    Serial.print("A2DP audio state: ");
    Serial.println(a2dp_source.to_str(a2dp_source.get_audio_state()));
    Serial.print("A2DP connection state: ");
    Serial.println(a2dp_source.to_str(a2dp_source.get_connection_state()));

    if (connected) {
        digitalWrite(BT_CONNECTED_PIN, HIGH);
    } else {
        digitalWrite(BT_CONNECTED_PIN, LOW);
    }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
    if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        Serial.println("Disconnected");
        selectedSSID = "";
        setConnected(false);
        sendNotification(NT_BT_DISCONNECTED);
    } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        Serial.println("Connected");
        connectionAttempts = 0;
        setConnected(true);
        sendNotification(NT_BT_CONNECTED);
    } else if (state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
        Serial.println("Connecting");
        sendNotification(NT_BT_CONNECTING);
    }
}

bool btDeviceIsValid(const char *ssid, esp_bd_addr_t address, int rssi) {
    (void) address;
    (void) rssi;
    const auto ssidString = std::string(ssid);
    if (!ssidString.empty()) {
        if (!selectedSSID.empty()
            && ssidString == selectedSSID) {
            return true;
        }
        if (btDevices.find(ssidString) == btDevices.end()) {
            btDevices[ssidString] = std::array<uint8_t, 6>{
                address[0],
                address[1],
                address[2],
                address[3],
                address[4],
                address[5]
            };
            sendNotification(NT_BT_DEVICE_LIST_CHANGED);
        }
    }
    return false;
}

void setupPins() {
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
    cfg.pin_bck = 26;
    cfg.pin_ws = 27;
    cfg.pin_data = 32;
    cfg.set(info44k1);
    i2s.begin(cfg);

    pinnacleInit();
    a2dp_source.set_local_name("SIDPod");
    a2dp_source.set_ssid_callback(btDeviceIsValid);
    a2dp_source.set_on_connection_state_changed(connection_state_changed);
    a2dp_source.set_data_source(i2s);
    a2dp_source.start();

    digitalWrite(BT_CONNECTED_PIN, LOW);
}

double toDegrees(const _absData *absData) {
    const int deltaX = absData->xValue - lastX;
    const int deltaY = absData->yValue - lastY;
    const double rad = atan2(deltaY, deltaX);
    double degrees = rad * 180 / M_PI;
    return degrees;
}

int getVerticalDelta(const _absData *absData) {
    return absData->yValue - lastY;
}

int getHorizontalDelta(const _absData *absData) {
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
    if (forcedGesture != G_NONE) {
        return forcedGesture;
    }
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

    int degreeLimit = static_cast<int>(degreeSorted.front().degreeValue < 0 ? 0.3 : 2) * deltaY;

    if (deltaDegrees >= degreeLimit
        && ySorted.back().yValue >= 170
        && ySorted.back().yValue <= 400
        && deltaY < 200) {
        return G_ROTATE;
    }
    if (std::abs(deltaX) > std::abs(deltaY)) {
        return G_HORIZONTAL;
    }
    return G_VERTICAL;
}

const char* getNotificationTypeName(NotificationType type) {
    switch (type) {
        case NT_NONE: return "NT_NONE";
        case NT_GESTURE: return "NT_GESTURE";
        case NT_BT_CONNECTED: return "NT_BT_CONNECTED";
        case NT_BT_DISCONNECTED: return "NT_BT_DISCONNECTED";
        case NT_BT_DEVICE_LIST_CHANGED: return "NT_BT_DEVICE_LIST_CHANGED";
        case NT_BT_CONNECTING: return "NT_BT_CONNECTING";
        default: return "UNKNOWN_NOTIFICATION";
    }
}

void sendNotification(const NotificationType notificationType) {
    Serial.print("Sending notification: ");
    Serial.println(getNotificationTypeName(notificationType));
    Serial1.write(notificationType);
}

void sendGesture(const Gesture gesture, const bool modifier = false) {
    Serial1.write(NT_GESTURE);
    Serial1.write(gesture);
    Serial1.write(modifier ? 1 : 0);
    Serial1.flush();
    delay(POST_GESTURE_DELAY_MS);
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

void forgetCurrentDevice() {
    selectedSSID = "";
    a2dp_source.disconnect();
    a2dp_source.clean_last_connection();
    a2dp_source.start();
}

void getCurrentDeviceAddress(esp_bd_addr_t addr) {
    auto addressArray = btDevices.find(selectedSSID)->second;
    std::copy(addressArray.begin(), addressArray.end(), addr);
}

void loop() {
    if (millis() - lastSessionEndTime.count() > 100 && drAsserted()) {
        pinnacleGetAbsolute(&touchData);
        if (!touchData.touchDown) {
            touchDowns++;
        }
        if (touchData.zValue > 20) {
            inSession = true;
            scaleData(&touchData, 1000, 1000);
            if (historyVector.size() > 5 && currentGesture == G_NONE) {
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
                       && (lastY > 800 || lastY < 200)) {
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
                sendGesture(G_NORTH);
            } else if (lastY > 666) {
                sendGesture(G_SOUTH);
            } else if (lastX < 333) {
                sendGesture(G_WEST);
            } else if (lastX > 666) {
                sendGesture(G_EAST);
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
        Serial1.flush();
        if (type == static_cast<char>(RT_BT_LIST) && !btDevices.empty()) {
            Serial.println("RT_BT_LIST");
            for (const auto &device: btDevices) {
                Serial1.println(device.first.c_str());
                Serial.print("'");
                Serial.print(device.first.c_str());
                Serial.println("'");
            }
        } else if (type == static_cast<char>(RT_BT_SELECT)) {
            selectedSSID = Serial1.readStringUntil('\n').c_str();
            selectedSSID = selectedSSID.substr(0, selectedSSID.length() - 1);
            getCurrentDeviceAddress(selectedAddress);
            setConnected(a2dp_source.connect_to(selectedAddress));
            Serial1.flush();
        } else if (type == static_cast<char>(RT_BT_DISCONNECT)) {
            forgetCurrentDevice();
        } else if (type == static_cast<char>(RT_BT_GET_CONNECTED)) {
            Serial1.println(selectedSSID.c_str());
            Serial1.write(selectedAddress, 6);
        } else if (type == static_cast<char>(RT_G_FORCE_ROTATE)) {
            Serial.println("RT_G_FORCE_ROTATE");
            forcedGesture = G_ROTATE;
        } else if (type == static_cast<char>(RT_G_FORCE_VERTICAL)) {
            Serial.println("RT_G_FORCE_VERTICAL");
            forcedGesture = G_VERTICAL;
        } else if (type == static_cast<char>(RT_G_FORCE_HORIZONTAL)) {
            forcedGesture = G_HORIZONTAL;
        } else if (type == static_cast<char>(RT_G_SET_AUTO)) {
            Serial.println("RT_G_SET_AUTO");
            forcedGesture = G_NONE;
        }
    }

    if (millis() - lastDeviceValidation.count() > DEVICE_VALIDATION_INTERVAL_MS) {
        btDevices.clear();
        lastDeviceValidation = std::chrono::milliseconds(millis());
    }
}
