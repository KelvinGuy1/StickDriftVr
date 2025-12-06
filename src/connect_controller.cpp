#include "connect_controller.h"
#include <iostream>
#include "openvr_driver.h"

hid_device *leftHIDHandle = nullptr;
hid_device *rightHIDHandle = nullptr;

bool HIDController::search_controller_left() {
    vr::VRDriverLog()->Log("Connecting left Joy-Con");
    struct hid_device_info *devs = hid_enumerate(0x057e, 0x0);
    for (struct hid_device_info *cur_dev = devs; cur_dev; cur_dev = cur_dev->next) {
        if (cur_dev->product_id == 0x2006) { // Left Joy-Con
            leftHIDHandle = hid_open_path(cur_dev->path);
            vr::VRDriverLog()->Log("Got left");
            break;
        }
    }
    hid_free_enumeration(devs);

    if (leftHIDHandle != nullptr) {
        return true;
    } else {
        vr::VRDriverLog()->Log("Failed to connect left Joy-Con");
        return false;
    }
}

bool HIDController::search_controller_right() {
    vr::VRDriverLog()->Log("Connecting right Joy-Con");
    struct hid_device_info *devs = hid_enumerate(0x057e, 0x0);
    for (struct hid_device_info *cur_dev = devs; cur_dev; cur_dev = cur_dev->next) {
        if (cur_dev->product_id == 0x2007) { // Right Joy-Con
            rightHIDHandle = hid_open_path(cur_dev->path);
            vr::VRDriverLog()->Log("Got right");
            break;
        }
    }
    hid_free_enumeration(devs);

    if (rightHIDHandle != nullptr) {
        return true;
    } else {
        vr::VRDriverLog()->Log("Failed to connect right Joy-Con");
        return false;
    }
}

void HIDController::freeControllers() {
    if (leftHIDHandle != nullptr) {
        hid_close(leftHIDHandle);
        leftHIDHandle = nullptr;
        vr::VRDriverLog()->Log( "Left controller freed" );
    }
    if (rightHIDHandle != nullptr) {
        hid_close(rightHIDHandle);
        rightHIDHandle = nullptr;
        vr::VRDriverLog()->Log( "Right controller freed" );
    }
}
