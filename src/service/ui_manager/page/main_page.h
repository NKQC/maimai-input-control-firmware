#pragma once

#include "../engine/page_construction/page_constructor.h"
#include <cstdint>

namespace ui {

/**
 * 主界面页面构造器
 * 显示系统运行时长和基本状态信息
 */
class MainPage : public PageConstructor {
public:
    MainPage();
    virtual ~MainPage() = default;
    
    /**
     * 渲染主界面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 设置当前页面名称（用于区分main和main_menu）
     * @param page_name 页面名称
     */
    void set_current_page_name(const std::string& page_name);
    
    /**
     * 更新运行时长（由外部调用）
     * @param uptime_ms 运行时长（毫秒）
     */
    void update_uptime(uint32_t uptime_ms);
    
    /**
     * 设置系统状态
     * @param status 状态字符串
     */
    void set_system_status(const std::string& status);
    
    /**
     * 设置连接状态
     * @param connected 是否连接
     */
    void set_connection_status(bool connected);
    
private:
    uint32_t uptime_ms_;           // 运行时长（毫秒）
    std::string system_status_;    // 系统状态
    bool connection_status_;       // 连接状态
    std::string current_page_name_; // 当前页面名称（用于区分main和main_menu）
    
    // 辅助函数
    std::string format_uptime(uint32_t uptime_ms) const;
    std::string get_connection_text() const;
};

} // namespace ui