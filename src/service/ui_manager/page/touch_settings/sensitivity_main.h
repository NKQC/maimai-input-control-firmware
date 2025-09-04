#pragma once

#include "../../engine/page_construction/page_constructor.h"
#include "../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 灵敏度调整主页面构造器
 * 显示所有IC设备列表和交互式调整选项
 */
class SensitivityMain : public PageConstructor {
public:
    SensitivityMain();
    virtual ~SensitivityMain() = default;
    
    /**
     * 渲染灵敏度调整主页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
private:
    // 无需额外的私有成员
};

} // namespace ui