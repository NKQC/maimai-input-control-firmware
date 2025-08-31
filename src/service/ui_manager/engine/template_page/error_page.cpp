#include "error_page.h"
#include "../page_construction/page_macros.h"

namespace ui {

ErrorPage::ErrorPage() 
    : error_message_("系统错误")
    , action_hint_("按任意键返回") {
}

void ErrorPage::render(PageTemplate& page_template) {
    // 从共享数据中获取错误信息（如果有的话）
    std::string error_msg = get_shared_data("error_message", error_message_);
    std::string action_hint = get_shared_data("action_hint", action_hint_);
    
    PAGE_START();
    
    // 设置标题
    SET_TITLE("错误", COLOR_ERROR);
    
    // 第一行：空行
    ADD_TEXT("", COLOR_TEXT_WHITE, LineAlign::CENTER);
    
    // 第二行：错误信息
    ADD_TEXT(error_msg, COLOR_ERROR, LineAlign::CENTER);
    
    // 第三行：空行
    ADD_TEXT("", COLOR_TEXT_WHITE, LineAlign::CENTER);
    
    ADD_TEXT("重启设备", COLOR_TEXT_WHITE, LineAlign::CENTER);
    PAGE_END();
}

void ErrorPage::set_error_info(const std::string& error_message, const std::string& action_hint) {
    error_message_ = error_message;
    action_hint_ = action_hint;
    
    // 同时设置到共享数据中，以便在页面切换时保持状态
    set_shared_data("error_message", error_message);
    set_shared_data("action_hint", action_hint);
}

} // namespace ui