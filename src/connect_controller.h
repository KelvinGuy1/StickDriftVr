#ifndef CONNECT_CONTROLLER_H
#define CONNECT_CONTROLLER_H

#include <hidapi.h>

class HIDController {
public:
    hid_device *leftHIDHandle;
    hid_device *rightHIDHandle;

    bool search_controller_left();
    bool search_controller_right();
    void freeControllers();
};

#endif // CONNECT_CONTROLLER_H
