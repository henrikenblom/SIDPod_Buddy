//
// Created by Henrik Enblom on 2025-04-20.
//

#ifndef BUDDYINTERFACE_H
#define BUDDYINTERFACE_H

#define SIDPOD_DR_PIN                  21
#define SIDPOD_DW_PIN             16
#define HORIZONTAL_PIN           26
#define ROTATE_PIN               13
#define MODIFIER1_PIN            12
#define MODIFIER2_PIN            27
#define BT_CONNECTED_PIN         4

enum RequestType {
    RT_NONE = 0,
    RT_BT_LIST = 1,
    RT_BT_SELECT = 2,
    RT_BT_DISCONNECT = 3,
    RT_G_FORCE_ROTATE = 4,
    RT_G_FORCE_VERTICAL = 5,
    RT_G_FORCE_HORIZONTAL = 6,
    RT_G_SET_AUTO = 7,
};

enum NotificationType {
    NT_NONE = 0,
    NT_GESTURE = 1,
    NT_BT_CONNECTED = 2,
    NT_BT_DISCONNECTED = 3,
    NT_BT_DEVICE_LIST_CHANGED = 4,
    NT_BT_CONNECTING = 5,
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

struct Request {
    RequestType type;
    char data[32];
};
#endif //BUDDYINTERFACE_H
