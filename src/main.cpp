#include <Arduino.h>
#include <chrono>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include "BuddyInterface.h"
#include "TM0XX0XX.h"
#include "main.h"

#include <map>

uint16_t lastX, lastY = 0;
bool inSession, gestureSent = false;
std::chrono::milliseconds lastMovementTime, lastStepUpdateTime, lastSessionEndTime, lastDeviceValidation;
int touchSamples, touchDowns, connectionAttempts = 0;
std::map<std::string, std::array<uint8_t, 6> > btDevices;
double accumulatedDelta, lastDegrees = 0;
std::string selectedSSID;
std::vector<bool> rotationHistory;

absData_t touchData;

AudioInfo info44k1(44100, 2, 16);
BluetoothA2DPSource a2dp_source;
I2SStream i2s;

constexpr int BYTES_PER_FRAME = 4;

int32_t getSoundData(Frame *data, const int32_t frameCount) {
    return static_cast<int32_t>(i2s.readBytes(reinterpret_cast<uint8_t *>(data), frameCount * BYTES_PER_FRAME)) /
           BYTES_PER_FRAME;
}

void setConnected(bool connected) {
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
    pinMode(SIDPOD_DW_PIN, OUTPUT);
    pinMode(SIDPOD_DR_PIN, INPUT);
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

    pinnacleInit();
    a2dp_source.set_ssid_callback(btDeviceIsValid);
    a2dp_source.set_on_connection_state_changed(connection_state_changed);
    a2dp_source.set_data_callback_in_frames(getSoundData);
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

double getRotationalDelta(double degrees) {
    double delta = degrees - lastDegrees;
    if (delta < -180) {
        delta += 360;
    } else if (delta > 180) {
        delta -= 360;
    }
    return delta;
}

void sendNotification(const NotificationType notificationType) {
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

void sendDeJitteredRotationGesture(const bool modifier = false) {
    rotationHistory.push_back(modifier);
    if (rotationHistory.size() > 4) {
        rotationHistory.erase(rotationHistory.begin());
    }
    if (rotationHistory.size() == 4
        && rotationHistory[0] != modifier
        && rotationHistory[1] != modifier
        && rotationHistory[2] != modifier
        && rotationHistory[3] != modifier) {
    } else {
        sendGesture(G_ROTATE, modifier);
    }
}

void forgetCurrentDevice() {
    selectedSSID = "";
    a2dp_source.disconnect();
    a2dp_source.clean_last_connection();
    a2dp_source.start();
}

void loop() {
    if (millis() - lastSessionEndTime.count() > 100 && drAsserted()) {
        pinnacleGetAbsolute(&touchData);
        if (!touchData.touchDown) {
            touchDowns++;
        }
        if (touchData.zValue > 30) {
            if (touchSamples > 5) {
                inSession = true;
                scaleData(&touchData, 1000, 1000);
                if ((touchData.xValue || touchData.yValue)
                    && (lastX != touchData.xValue || lastY != touchData.yValue)) {
                    const double degrees = toDegrees(&touchData);
                    accumulatedDelta += getRotationalDelta(degrees);
                    lastDegrees = degrees;
                }
                lastX = touchData.xValue;
                lastY = touchData.yValue;
                lastMovementTime = std::chrono::milliseconds(millis());
                if (touchSamples >= 20
                    && (accumulatedDelta > ROTATE_STEP_SIZE || accumulatedDelta < ROTATE_STEP_SIZE * -1)) {
                    sendDeJitteredRotationGesture(accumulatedDelta > 0);
                    accumulatedDelta = 0;
                    lastStepUpdateTime = std::chrono::milliseconds(millis());
                }
            }
            touchSamples++;
        }
    } else if (inSession && millis() - lastMovementTime.count() > 180) {
        Serial.print("gestureSent: ");
        Serial.println(gestureSent);
        Serial.print("touchDowns: ");
        Serial.println(touchDowns);
        Serial.print("touchSamples: ");
        Serial.println(touchSamples);
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
        gestureSent = false;
        lastSessionEndTime = std::chrono::milliseconds(millis());
        rotationHistory.clear();
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
            auto addressArray = btDevices.find(selectedSSID)->second;
            esp_bd_addr_t addr = {
                addressArray[0],
                addressArray[1],
                addressArray[2],
                addressArray[3],
                addressArray[4],
                addressArray[5]
            };
            setConnected(a2dp_source.connect_to(addr));
            Serial1.flush();
        } else if (type == static_cast<char>(RT_BT_DISCONNECT)) {
            forgetCurrentDevice();
        }
    }

    if (millis() - lastDeviceValidation.count() > DEVICE_VALIDATION_INTERVAL_MS) {
        btDevices.clear();
        lastDeviceValidation = std::chrono::milliseconds(millis());
    }
}
