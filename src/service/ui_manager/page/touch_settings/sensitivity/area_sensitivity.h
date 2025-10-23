#ifndef AREA_SENSITIVITY_H
#define AREA_SENSITIVITY_H

#include "../../../../input_manager/input_manager.h"
#include "../../../../../protocol/mai2serial/mai2serial.h"
#include "../../../engine/page_construction/page_constructor.h"
#include <vector>
#include <string>

namespace ui {

class AreaSensitivity : public PageConstructor {
public:
    // 区域信息结构 - 前置声明
    struct AreaInfo {
        uint8_t area_index;
        std::string name;
        uint32_t channel_id;
        uint8_t device_mask;
        uint8_t channel;
        bool is_bound;
        bool supports_sensitivity;
        bool is_relative_mode;
        int32_t current_value;
        
        // 新的简化结构
        std::string area_name;
        uint8_t current_value_u8;
        bool has_modified;
    };
    
    // 区域组信息结构
    struct ZoneInfo {
        uint8_t zone_index;
        std::string zone_name;
        AreaInfo areas[8];  // 每个zone固定8个区域
        bool has_any_bindings;
    };

    AreaSensitivity();
    virtual ~AreaSensitivity() = default;
    
    // 主页面：显示A-E区选择
    virtual void render(PageTemplate& page_template) override;
    
    // 子页面：显示具体区域的灵敏度设置
    void renderZoneDetail(PageTemplate& page_template, uint8_t zone_index);
    void renderAreaDetail(PageTemplate& page_template);
    
    // 获取区域绑定信息 - 使用固定大小数组
    void getAreaBindingInfo(AreaInfo* areas, uint8_t max_areas, uint8_t* area_count);
    
    /**
     * 处理字符串参数传递
     */
    void jump_str(const std::string& str) override;
    
    /**
     * 加载区域数据
     */
    void loadAreaData();
    
    // 当前区域参数
    std::string current_area_param_;

private:
    static ZoneInfo s_zone_infos_[5];  // 固定5个区域组 A-E
    static bool s_initialized_;
    static uint8_t s_current_zone_index_;
    static int32_t s_current_area_index_;
    
    // 静态方法，供其他页面访问
public:
    static ZoneInfo* getZoneInfos() { return s_zone_infos_; }
    
    // 初始化区域信息
    static void init_zone_infos();
    
    // 获取区域索引 (A=0, B=1, C=2, D=3, E=4)
    static uint8_t getZoneIndex(uint8_t area_index);
    
    // 获取区域名称
    static std::string getAreaName(uint8_t area_index);
    
    // 获取区域绑定的通道ID
    static uint32_t getAreaChannelId(uint8_t area_index);
    
    // 获取分区名称
    static std::string getZoneName(uint8_t zone_index);
    
    // 回调函数
    static void on_sensitivity_change(int32_t new_value);
    static void on_sensitivity_complete();
    
    // 区域导航回调
    static void on_zone_select(uint8_t zone_index);
};

} // namespace ui

#endif // AREA_SENSITIVITY_H