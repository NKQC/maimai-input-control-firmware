#pragma once

#include "../engine/page_construction/page_constructor.h"
#include <cstdint>

namespace ui {

/**
 * 主菜单页面构造器
 * 包含进度条测试功能
 */
class MainMenu : public PageConstructor {
public:
    MainMenu();
    virtual ~MainMenu() = default;
    
    /**
     * 渲染主菜单页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 设置进度条进度
     * @param progress 进度值 (0-100)
     */
    void set_progress(int32_t progress);
    
    /**
     * 获取当前进度
     * @return 当前进度值 (0-100)
     */
    int32_t get_progress() const;

    void save_config();
    
private:
    int32_t progress_;  // 进度条进度 (0-100)
    uint8_t progress_data_;  // 进度条数据 (0-255)，用于ADD_PROGRESS宏

    
};

} // namespace ui