//
// Created by Henrik Enblom on 2025-04-22.
//

#ifndef MAIN_H
#define MAIN_H

#define VERTICAL_STEP_SIZE              55
#define HORIZONTAL_STEP_SIZE            40
#define ROTATE_STEP_SIZE                15
#define DEVICE_VALIDATION_INTERVAL_MS   20000

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


void connection_state_changed(esp_a2d_connection_state_t state, void *ptr);
bool btDeviceIsValid(const char *ssid, esp_bd_addr_t address, int rssi);
void setupPins();
void setup();
void loop();
void get_sound_data(uint8_t *buffer, size_t size);
void bang(uint8_t pin, bool modifier1 = false, bool modifier2 = false);
void forgetCurrentDevice();


#endif //MAIN_H
