//
// Created by Henrik Enblom on 2025-04-22.
//

#ifndef MAIN_H
#define MAIN_H

#define ROTATE_STEP_SIZE                50
#define DEVICE_VALIDATION_INTERVAL_MS   40000
#define POST_GESTURE_DELAY_MS           150
#include "BuddyInterface.h"

struct _historyData {
    uint16_t xValue;
    uint16_t yValue;
    double degreeValue;
};

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr);
bool btDeviceIsValid(const char *ssid, const esp_bd_addr_t address, int rssi);
void setupPins();
void setup();
void loop();
void get_sound_data(uint8_t *buffer, size_t size);
void sendNotification(NotificationType notificationType);
void forgetCurrentDevice();


#endif //MAIN_H
