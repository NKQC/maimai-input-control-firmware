#pragma once

#include "engine/page_construction/page_constructor.h"
#include <string>
#include <map>
#include <memory>

// 页面注册宏，简化注册过程
#define REGISTER_PAGE(page_name, page_class) \
    do { \
        auto page_instance = std::make_shared<page_class>(); \
        register_page(page_name, page_instance); \
    } while(0)

namespace ui {

/**
 * 页面注册管理器
 * 统一管理所有页面构造器的注册和获取
 */
class PageRegistry {
public:
    // 获取单例实例
    static PageRegistry& get_instance();
    
    /**
     * 注册页面构造器
     * @param page_name 页面名称
     * @param constructor 页面构造器智能指针
     * @return 是否注册成功
     */
    bool register_page(const std::string& page_name, std::shared_ptr<PageConstructor> constructor);
    
    /**
     * 获取页面构造器
     * @param page_name 页面名称
     * @return 页面构造器智能指针，如果不存在则返回nullptr
     */
    std::shared_ptr<PageConstructor> get_page(const std::string& page_name);
    
    /**
     * 检查页面是否存在
     * @param page_name 页面名称
     * @return 是否存在
     */
    bool has_page(const std::string& page_name) const;
    
    /**
     * 移除页面
     * @param page_name 页面名称
     * @return 是否移除成功
     */
    bool unregister_page(const std::string& page_name);
    
    /**
     * 清空所有注册的页面
     */
    void clear_all_pages();
    
    /**
     * 获取已注册页面数量
     * @return 页面数量
     */
    size_t get_page_count() const;
    
    /**
     * 获取所有页面名称
     * @return 页面名称列表
     */
    std::vector<std::string> get_all_page_names() const;
    
    /**
     * 注册所有默认页面
     */
    void register_default_pages();
    
private:
    // 私有构造函数（单例模式）
    PageRegistry() = default;
    ~PageRegistry() = default;
    PageRegistry(const PageRegistry&) = delete;
    PageRegistry& operator=(const PageRegistry&) = delete;
    
    // 页面存储映射
    std::map<std::string, std::shared_ptr<PageConstructor>> pages_;
};

} // namespace ui