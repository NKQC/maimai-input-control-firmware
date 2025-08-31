#include "selector_test.h"
#include "../engine/page_construction/page_macros.h"
#include "../../../protocol/usb_serial_logs/usb_serial_logs.h"
#include "../ui_manager.h"
#include <string>

namespace ui {

SelectorTest::SelectorTest() : test_value_(0) {
}

void SelectorTest::render(PageTemplate& page_template) {
    PAGE_START()
    
    // 页面标题
    ADD_TEXT("选择器功能测试", COLOR_WHITE, TextAlign::CENTER)
    ADD_TEXT("", COLOR_WHITE, TextAlign::LEFT) // 空行
    
    // 测试值显示
    std::string value_text = "当前值: " + std::to_string(test_value_);
    ADD_TEXT(value_text, COLOR_TEXT_WHITE, TextAlign::LEFT)
    ADD_TEXT("", COLOR_WHITE, TextAlign::LEFT) // 空行
    
    // 选择器项 - 使用lambda回调
    ADD_SELECTOR("数值调节器 (按确认锁定)", 
                [this](JoystickState state) {
                    switch (state) {
                        case JoystickState::UP:
                            test_value_++;
                            USB_LOG_DEBUG("Selector UP: value = %d", test_value_);
                            break;
                        case JoystickState::DOWN:
                            test_value_--;
                            USB_LOG_DEBUG("Selector DOWN: value = %d", test_value_);
                            break;
                        case JoystickState::CONFIRM:
                            USB_LOG_DEBUG("Selector CONFIRM: value = %d", test_value_);
                            break;
                    }
                },
                [this]() {
                    USB_LOG_DEBUG("Selector lock state changed");
                },
                COLOR_TEXT_WHITE, TextAlign::LEFT)
    
    ADD_TEXT("", COLOR_WHITE, TextAlign::LEFT) // 空行
    ADD_TEXT("操作说明:", COLOR_YELLOW, TextAlign::LEFT)
    ADD_TEXT("1. 选中选择器项按确认键锁定", COLOR_TEXT_WHITE, TextAlign::LEFT)
    ADD_TEXT("2. 锁定后上下摇杆调节数值", COLOR_TEXT_WHITE, TextAlign::LEFT)
    ADD_TEXT("3. 再次按确认键解锁", COLOR_TEXT_WHITE, TextAlign::LEFT)
    ADD_TEXT("", COLOR_WHITE, TextAlign::LEFT) // 空行
    
    // 返回按钮
    ADD_BACK_ITEM("返回主菜单", COLOR_TEXT_WHITE)
    
    PAGE_END()
    
    // 绘制页面
    page_template.draw();
}

int SelectorTest::get_test_value() const {
    return test_value_;
}

void SelectorTest::set_test_value(int value) {
    test_value_ = value;
}

} // namespace ui