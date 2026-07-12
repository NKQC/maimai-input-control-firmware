#pragma once

/**
 * UsbComm - USB 通信服务骨架（单例）
 * 3xCDC 逻辑延后到后续里程碑，当前只提供空实现占位。
 */
class UsbComm {
public:
    static UsbComm* getInstance();

    bool init();
    void update();

private:
    UsbComm();
    UsbComm(const UsbComm&) = delete;
    UsbComm& operator=(const UsbComm&) = delete;

    static UsbComm* _instance;
};
