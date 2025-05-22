#include <Arduino.h>
#include <chrono>

#include "AudioTools.h"
#include "AudioTools/AudioLibs/A2DPStream.h"
#include "BuddyInterface.h"
#include "TM0XX0XX.h"
#include "main.h"

#include <TensorFlowLite_ESP32.h> // Specific header for ESP32 (often included by lib)
#include <tensorflow/lite/micro/all_ops_resolver.h> // Or use MicroMutableOpResolver for smaller code
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/system_setup.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h> // If using a custom resolver
#include "model.h"

#include <map>

#include "tensorflow/lite/micro/micro_error_reporter.h"

// Constants for the model
const int kModelInputWidth = 28;
const int kModelInputHeight = 28;
const int kModelInputChannels = 1; // Grayscale
const int kModelOutputClasses = 26; // Uppercase A-Z

// An area of memory to use for input, output, and intermediate arrays.
// ESP32 has different memory regions. It's best to allocate in PSRAM if available,
// or a larger chunk of internal SRAM.
// PSRAM usage: DRAM_ATTR is for DRAM, but for PSRAM, use `SPIRAM_ATTR` on supported boards.
// If your ESP32 has PSRAM and you want to use it:
// #include <esp_heap_caps.h> // Needed for PSRAM
// uint8_t* tensor_arena = NULL; // Declare as pointer
// In setup():
//   tensor_arena = (uint8_t*)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//   if (tensor_arena == NULL) { Serial.println("Failed to allocate PSRAM tensor arena!"); while(1); }
// Otherwise, for internal SRAM:
const int kTensorArenaSize = 40 * 1024; // 40 KB - Adjust based on your model size
uint8_t tensor_arena[kTensorArenaSize];

// Pointers to the model, interpreter, and tensors
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *model_input = nullptr;
TfLiteTensor *model_output = nullptr;
const tflite::Model *model = nullptr;

// Our labels (A-Z)
const char *kCategoryLabels[] = {
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M",
    "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z"
};

uint16_t lastX, lastY = 0;
bool inSession, lastAccumulatedDeltaGTZero, gestureSent, scribbling = false;
Gesture currentGesture, forcedGesture = G_NONE;
std::chrono::milliseconds lastMovementTime, lastStepUpdateTime, lastSessionEndTime, lastDeviceValidation;
int gestureTouchSamples, touchDowns, currentStepSize, connectionAttempts = 0;
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
    setCpuFrequencyMhz(240);
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

    model = tflite::GetModel(emnist_uppercase_quant_model_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("Model schema mismatch! Expected %d, got %d",
                    TFLITE_SCHEMA_VERSION, model->version());
        while (1);
    }

    // Using AllOpsResolver for simplicity. For smaller footprints, use MicroMutableOpResolver
    // and explicitly add operations present in your model (e.g., AddConv2D, AddMaxPool2D).
    static tflite::AllOpsResolver resolver;

    // Create a MicroErrorReporter instance for error reporting
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter *error_reporter = &micro_error_reporter;

    // Build an interpreter to run the model with.
    static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
    interpreter = &static_interpreter;

    // Allocate tensors from the provided arena.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed! Required: %d bytes, Provided: %d bytes",
                    interpreter->arena_used_bytes(), kTensorArenaSize);
        while (1);
    }

    // Get information about the model's input and output tensors.
    model_input = interpreter->input(0);
    model_output = interpreter->output(0);

    // Verify input/output tensor types for quantized model
    if (model_input->type != kTfLiteInt8 || model_output->type != kTfLiteInt8) {
        MicroPrintf("Input/Output tensor types are not INT8. Expected a quantized model.");
        while (1);
    }

    MicroPrintf("TensorFlow Lite Micro initialized successfully!");
    MicroPrintf("Input tensor bytes: %d", model_input->bytes);
    MicroPrintf("Output tensor bytes: %d", model_output->bytes);
    MicroPrintf("Input Quant: scale=%.6f, zero_point=%d", (double) model_input->params.scale,
                model_input->params.zero_point);
    MicroPrintf("Output Quant: scale=%.6f, zero_point=%d", (double) model_output->params.scale,
                model_output->params.zero_point);

    scribbling = true;
    std::fill_n(model_input->data.int8, 28 * 28, -128);
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

const char *getNotificationTypeName(NotificationType type) {
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
        if (touchData.zValue > 5) {
            inSession = true;
            if (scribbling) {
                scaleData(&touchData, 28, 28);
                // Draw a 2x2 stroke centered at the touch point
                for (int dx = -1; dx <= 0; ++dx) {
                    for (int dy = -1; dy <= 0; ++dy) {
                        int x = touchData.xValue + dx;
                        int y = touchData.yValue + dy;
                        if (x >= 0 && x < 28 && y >= 0 && y < 28) {
                            model_input->data.int8[x + y * 28] = 127;
                        }
                    }
                }
            } else {
                scaleData(&touchData, 1000, 1000);
                if (historyVector.size() > 5 && currentGesture == G_NONE) {
                    currentGesture = identifyGesture(historyVector);
                    currentStepSize = getStepSizeForCurrentGesture();
                }
                if ((touchData.xValue || touchData.yValue)
                    && (lastX != touchData.xValue || lastY != touchData.yValue)) {
                    const double degrees = toDegrees(&touchData);
                    if (gestureTouchSamples > 6 && currentGesture == G_NONE) {
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
                gestureTouchSamples++;
            }
            lastMovementTime = std::chrono::milliseconds(millis());
        }
    } else if (inSession && millis() - lastMovementTime.count() > 2000) {
        inSession = false;
        if (scribbling) {
            int x = 0;
            for (int i = 0; i < 28 * 28; i++) {
                Serial.printf("%04d,", model_input->data.int8[i]);
                if (x++ > 26) {
                    Serial.println();
                    x = 0;
                }
            }
            Serial.println();

            // --- Input Data (Copy your preprocessed, quantized image into the input tensor) ---
            // The dummy_input_image is already int8 and sized correctly (28x28x1).
            //memcpy(model_input->data.int8, dummy_input_image, sizeof(dummy_input_image));

            // --- Run Inference ---
            TfLiteStatus invoke_status = interpreter->Invoke();
            if (invoke_status != kTfLiteOk) {
                MicroPrintf("Invoke failed!");
                return;
            }

            // --- Get Output and Dequantize ---
            // The output tensor holds quantized int8 values. We need to dequantize them.
            // Dequantization formula: float_value = (quantized_value - zero_point) * scale
            float output_values[kModelOutputClasses];
            int8_t* output_data = tflite::GetTensorData<int8_t>(model_output); // Correct way to get data pointer

            float output_scale = model_output->params.scale;
            int32_t output_zero_point = model_output->params.zero_point;

            for (int i = 0; i < kModelOutputClasses; ++i) {
                output_values[i] = (output_data[i] - output_zero_point) * output_scale;
            }

            // --- Find the Predicted Class ---
            int max_index = -1;
            float max_value = -1e9; // A very small number

            for (int i = 0; i < kModelOutputClasses; ++i) {
                if (output_values[i] > max_value) {
                    max_value = output_values[i];
                    max_index = i;
                }
            }

            // --- Print Results ---
            MicroPrintf("Prediction: %s", kCategoryLabels[max_index]);
            MicroPrintf("Confidence: %.4f", (double)max_value); // Use (double) for MicroPrintf with float

            // Optional: Print all probabilities for debugging
            // MicroPrintf("Probabilities:");
            // for (int i = 0; i < kModelOutputClasses; ++i) {
            //     MicroPrintf("  %s: %.4f", kCategoryLabels[i], (double)output_values[i]);
            // }


        } else {
            if (gestureTouchSamples > 3 && gestureTouchSamples < 20 && !gestureSent) {
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
        }
        accumulatedDelta = 0;
        gestureTouchSamples = 0;
        touchDowns = 0;
        lastY = 0;
        lastX = 0;
        lastDegrees = 0;
        historyVector.clear();
        currentGesture = G_NONE;
        gestureSent = false;
        lastSessionEndTime = std::chrono::milliseconds(millis());
        std::fill_n(model_input->data.int8, 28 * 28, -128);
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
