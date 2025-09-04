#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include <cstdint>

namespace ui {

/**
 * 触摸设置主页面构造器
 * 包含触摸IC状态和灵敏度调整菜单项
 */
class TouchSettingsMain : public PageConstructor {
public:
    TouchSettingsMain();
    virtual ~TouchSettingsMain() = default;
    
    /**
     * 渲染触摸设置主页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    static int32_t delay_value;
};

} // namespace ui