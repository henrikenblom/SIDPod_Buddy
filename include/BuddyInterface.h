//
// Created by Henrik Enblom on 2025-04-20.
//

#ifndef BUDDYINTERFACE_H
#define BUDDYINTERFACE_H

#define TAP_PIN                  21
#define VERTICAL_PIN             16
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
};

struct Request {
    RequestType type;
    char data[32];
};
#endif //BUDDYINTERFACE_H
