#pragma once

#include "../../../engine/page_construction/page_constructor.h"
#include "../../../../input_manager/input_manager.h"
#include <cstdint>
#include <string>

namespace ui {

/**
 * 单个区域灵敏度详细设置页面构造器
 * 显示特定区域的灵敏度调整选项
 */
class AreaSensitivityDetail : public PageConstructor {
public:
    AreaSensitivityDetail();
    virtual ~AreaSensitivityDetail() = default;
    
    /**
     * 渲染区域灵敏度详细设置页面
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) override;
    
    /**
     * 处理字符串参数传递
     */
    void jump_str(const std::string& str) override;

private:
    // 当前区域索引
    uint8_t current_area_index_;
    
    /**
     * 获取区域名称
     * @param area_index 区域索引 (1-34)
     * @return 区域名称字符串
     */
    std::string getAreaName(uint8_t area_index);
    
    /**
     * 获取当前区域的灵敏度值
     * @return 灵敏度值
     */
    int32_t getCurrentSensitivity();
    
    /**
     * 设置当前区域的灵敏度值
     * @param value 新的灵敏度值
     */
    void setSensitivity(int32_t value);
    
    /**
     * 检查当前区域是否支持灵敏度调整
     * @return true如果支持，false如果不支持
     */
    bool supportsSensitivity();
    
    /**
     * 静态完成回调函数，避免使用this指针
     */
    static void on_sensitivity_complete();

private:
    // 静态成员用于存储当前操作的区域信息
    static uint8_t current_area_index_static_;
    static int32_t current_sensitivity_value_;
};

} // namespace ui