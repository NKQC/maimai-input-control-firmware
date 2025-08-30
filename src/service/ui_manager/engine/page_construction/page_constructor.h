#pragma once

#include "page_template.h"
#include <string>
#include <map>
#include <memory>

namespace ui {

/**
 * 页面构造器抽象基类
 * 提供统一的页面渲染接口和页面间数据共享机制
 */
class PageConstructor {
public:
    // 虚析构函数
    virtual ~PageConstructor() = default;
    
    /**
     * 主渲染接口 - 纯虚函数
     * @param page_template PageTemplate实例引用
     */
    virtual void render(PageTemplate& page_template) = 0;
    
    /**
     * 设置页面上下文信息
     * @param page_name 当前页面名称
     * @param current_time 当前时间戳
     */
    virtual void set_page_context(const std::string& page_name, uint32_t current_time) {
        // 默认实现为空，子类可以重写此方法来接收上下文信息
    }
    
    // 页面间数据共享接口
    /**
     * 写入共享数据
     * @param key 数据键
     * @param value 数据值
     */
    static void set_shared_data(const std::string& key, const std::string& value);
    
    /**
     * 读取共享数据
     * @param key 数据键
     * @param default_value 默认值（当键不存在时返回）
     * @return 数据值
     */
    static std::string get_shared_data(const std::string& key, const std::string& default_value = "");
    
    /**
     * 移除共享数据
     * @param key 数据键
     * @return 是否成功移除
     */
    static bool remove_shared_data(const std::string& key);
    
    /**
     * 检查共享数据是否存在
     * @param key 数据键
     * @return 是否存在
     */
    static bool has_shared_data(const std::string& key);
    
    /**
     * 清空所有共享数据
     */
    static void clear_shared_data();
    
    /**
     * 获取共享数据数量
     * @return 数据项数量
     */
    static size_t get_shared_data_count();
    
protected:
    // 受保护的构造函数，防止直接实例化
    PageConstructor() = default;
    
private:
    // 静态共享数据存储
    static std::map<std::string, std::string> shared_data_map_;
};

} // namespace ui