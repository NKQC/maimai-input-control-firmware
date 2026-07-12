#include "usb_comm.h"

UsbComm* UsbComm::_instance = nullptr;

UsbComm::UsbComm() {}

UsbComm* UsbComm::getInstance() {
    if (!_instance) {
        _instance = new UsbComm();
    }
    return _instance;
}

bool UsbComm::init() {
    // 3xCDC 逻辑延后到后续里程碑
    return true;
}

void UsbComm::update() {
    // 3xCDC 逻辑延后到后续里程碑
}
