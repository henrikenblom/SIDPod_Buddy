//
// Created by Henrik Enblom on 2025-04-20.
//

#ifndef BUDDYINTERFACE_H
#define BUDDYINTERFACE_H

#define BT_CONNECTED_PIN            4
#define RX_PIN                      25
#define TX_PIN                      33
#define I2S_BCK_PIN                 35
#define I2S_WS_PIN                  34
#define I2S_DATA_PIN                32

enum RequestType {
    RT_NONE = 0,
    RT_BT_LIST = 1,
    RT_BT_SELECT = 2,
    RT_BT_DISCONNECT = 3,
    RT_G_FORCE_ROTATE = 4,
    RT_G_FORCE_VERTICAL = 5,
    RT_G_FORCE_HORIZONTAL = 6,
    RT_G_SET_AUTO = 7,
    RT_BT_GET_CONNECTED = 8,
    RT_SCRIBBLE = 9,
};

enum NotificationType {
    NT_NONE = 0,
    NT_GESTURE = 1,
    NT_BT_CONNECTED = 2,
    NT_BT_DISCONNECTED = 3,
    NT_BT_DEVICE_LIST_CHANGED = 4,
    NT_BT_CONNECTING = 5,
    NT_SCRIBBLE_INPUT = 6,
    NT_CHARACTER_DETECTED = 7,
    NT_BACKSPACE_DETECTED = 8,
    NT_SPACE_DETECTED = 9,
};

enum Gesture {
    G_NONE = 0,
    G_HORIZONTAL = 1,
    G_VERTICAL = 2,
    G_ROTATE = 3,
    G_TAP = 4,
    G_DOUBLE_TAP = 5,
    G_NORTH = 6,
    G_EAST = 7,
    G_WEST = 8,
    G_SOUTH = 9,
};

struct Request {
    RequestType type;
    char data[32];
};
#endif //BUDDYINTERFACE_H
