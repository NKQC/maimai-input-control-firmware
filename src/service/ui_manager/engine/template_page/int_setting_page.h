#pragma once

#include "../page_construction/page_constructor.h"
#include <string>
#include <functional>
#include <cstdint>

namespace ui {

/**
 * INT设置页面构造器
 * 用于数值设置的通用页面
 */
class IntSettingPage : public PageConstructor {
public:
    IntSettingPage();
    virtual ~IntSettingPage() = default;
    
    /**
     * 渲染INT设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 设置页面参数
     * @param title 页面标题
     * @param value_ptr 数值指针
     * @param min_val 最小值
     * @param max_val 最大值
     * @param change_cb 数值变化回调
     * @param complete_cb 完成回调
     */
    void setup_data(const std::string& title, 
               int32_t* value_ptr, int32_t min_val, int32_t max_val,
               std::function<void(int32_t)> change_cb = nullptr,
               std::function<void()> complete_cb = nullptr);
    
private:
    std::string title_;
    int32_t* value_ptr_;
    int32_t min_val_;
    int32_t max_val_;
    std::function<void(int32_t)> change_cb_;
    std::function<void()> complete_cb_;
    
    // 辅助函数
    std::string format_value_display() const;
    std::string format_range_display() const;
};

} // namespace ui