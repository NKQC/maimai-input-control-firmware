#pragma once

/**
 * PsocUpdater - PSoC 固件版本比对 + SWD 烧录服务骨架（单例）
 * 版本比对 + SWD 烧录逻辑延后到后续里程碑，当前只提供空实现占位。
 */
class PsocUpdater {
public:
    static PsocUpdater* getInstance();

    bool init();
    void update();

private:
    PsocUpdater();
    PsocUpdater(const PsocUpdater&) = delete;
    PsocUpdater& operator=(const PsocUpdater&) = delete;

    static PsocUpdater* _instance;
};
