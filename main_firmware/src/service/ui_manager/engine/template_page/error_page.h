#pragma once

#include "../page_construction/page_constructor.h"
#include <string>

namespace ui {

/**
 * 错误页面构造器
 * 显示错误信息和操作提示
 */
class ErrorPage : public PageConstructor {
public:
    ErrorPage();
    virtual ~ErrorPage() = default;
    
    /**
     * 渲染错误页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 设置错误信息
     * @param error_message 错误信息
     * @param action_hint 操作提示
     */
    void set_error_info(const std::string& error_message, const std::string& action_hint = "按任意键返回");
    
private:
    std::string error_message_;
    std::string action_hint_;
};

} // namespace ui