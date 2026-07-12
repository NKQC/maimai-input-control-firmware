#include "psoc_updater.h"

PsocUpdater* PsocUpdater::_instance = nullptr;

PsocUpdater::PsocUpdater() {}

PsocUpdater* PsocUpdater::getInstance() {
    if (!_instance) {
        _instance = new PsocUpdater();
    }
    return _instance;
}

bool PsocUpdater::init() {
    // 版本比对 + SWD 烧录逻辑延后到后续里程碑
    return true;
}

void PsocUpdater::update() {
    // 版本比对 + SWD 烧录逻辑延后到后续里程碑
}
