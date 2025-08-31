#include "int_setting_page.h"
#include "../page_construction/page_macros.h"
#include <sstream>

namespace ui {

IntSettingPage::IntSettingPage() 
    : title_("设置")
    , value_ptr_(nullptr)
    , min_val_(0)
    , max_val_(100)
    , change_cb_(nullptr)
    , complete_cb_(nullptr) {
}

void IntSettingPage::render(PageTemplate& page_template) {
    // 从共享数据中获取设置参数（如果有的话）
    std::string title = get_shared_data("int_setting_title", title_);
    
    PAGE_START();
    
    // 设置标题
    SET_TITLE(title, COLOR_WHITE);
    
    // 第一行: 空行
    ADD_TEXT("", COLOR_TEXT_WHITE, LineAlign::CENTER);
    
    // 第二行: 当前值
    ADD_TEXT(format_value_display(), COLOR_TEXT_WHITE, LineAlign::CENTER);

    // 第三行: 范围
    ADD_TEXT(format_range_display(), COLOR_TEXT_WHITE, LineAlign::CENTER);

    // 第四行：操作提示
    ADD_TEXT("↑↓ 调整值  确认键返回", COLOR_TEXT_WHITE, LineAlign::CENTER);
    
    PAGE_END();
}

void IntSettingPage::setup_data(const std::string& title, 
                          int32_t* value_ptr, int32_t min_val, int32_t max_val,
                          std::function<void(int32_t)> change_cb,
                          std::function<void()> complete_cb) {
    title_ = title;
    value_ptr_ = value_ptr;
    min_val_ = min_val;
    max_val_ = max_val;
    change_cb_ = change_cb;
    complete_cb_ = complete_cb;
}

std::string IntSettingPage::format_value_display() const {
    if (value_ptr_) {
        return std::to_string(*value_ptr_);
    }
    return "--";
}

std::string IntSettingPage::format_range_display() const {
    std::ostringstream oss;
    oss << min_val_ << " - " << max_val_;
    return oss.str();
}

} // namespace ui