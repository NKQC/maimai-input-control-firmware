#include "page_registry.h"
#include "page/main_page.h"
#include "page/main_menu.h"
#include "page/touch_settings/touch_settings_main.h"
#include "page/touch_settings/sensitivity_main.h"
#include "page/touch_settings/sensitivity_device.h"
#include "page/touch_settings/interactive_sensitivity.h"
#include "page/touch_settings/device_custom_settings/ad7147_custom_settings.h"
#include "page/binding_settings/binding_settings.h"
#include "page/binding_settings/binding_info.h"
#include "page/general_settings/general_settings.h"
#include "page/communication_settings/communication_settings.h"
#include "engine/template_page/error_page.h"
#include "engine/template_page/int_setting_page.h"
#include <algorithm>

namespace ui {

PageRegistry& PageRegistry::get_instance() {
    static PageRegistry instance;
    return instance;
}

bool PageRegistry::register_page(const std::string& page_name, std::shared_ptr<PageConstructor> constructor) {
    if (page_name.empty() || !constructor) {
        return false;
    }
    
    pages_[page_name] = constructor;
    return true;
}

std::shared_ptr<PageConstructor> PageRegistry::get_page(const std::string& page_name) {
    auto it = pages_.find(page_name);
    if (it != pages_.end()) {
        return it->second;
    }
    return nullptr;
}

bool PageRegistry::has_page(const std::string& page_name) const {
    return pages_.find(page_name) != pages_.end();
}

bool PageRegistry::unregister_page(const std::string& page_name) {
    auto it = pages_.find(page_name);
    if (it != pages_.end()) {
        pages_.erase(it);
        return true;
    }
    return false;
}

void PageRegistry::clear_all_pages() {
    pages_.clear();
}

size_t PageRegistry::get_page_count() const {
    return pages_.size();
}

std::vector<std::string> PageRegistry::get_all_page_names() const {
    std::vector<std::string> names;
    names.reserve(pages_.size());
    
    for (const auto& pair : pages_) {
        names.push_back(pair.first);
    }
    
    return names;
}

void PageRegistry::register_default_pages() {
    // 注册主页面
    auto main_page = std::make_shared<MainPage>();
    register_page("main", main_page);
    
    // 注册主菜单页面
    auto main_menu_page = std::make_shared<MainMenu>();
    register_page("main_menu", main_menu_page);
    
    // 注册触摸设置相关页面
    auto touch_settings_main_page = std::make_shared<TouchSettingsMain>();
    register_page("touch_settings_main", touch_settings_main_page);
    
    auto sensitivity_main_page = std::make_shared<SensitivityMain>();
    register_page("sensitivity_main", sensitivity_main_page);
    
    auto sensitivity_device_page = std::make_shared<SensitivityDevice>();
    register_page("sensitivity_device", sensitivity_device_page);
    
    auto interactive_sensitivity_page = std::make_shared<InteractiveSensitivity>();
    register_page("interactive_sensitivity", interactive_sensitivity_page);
    
    // 注册AD7147自定义设置页面
    auto ad7147_custom_settings_page = std::make_shared<AD7147CustomSettings>();
    register_page("ad7147_custom_settings", ad7147_custom_settings_page);
    
    // 注册绑区设置页面
    auto binding_settings_page = std::make_shared<BindingSettings>();
    register_page("binding_settings", binding_settings_page);
    
    // 注册绑区信息显示页面
    auto binding_info_page = std::make_shared<BindingInfo>();
    register_page("binding_info", binding_info_page);
    
    // 注册通用设置页面
    auto general_settings_page = std::make_shared<GeneralSettings>();
    register_page("general_settings", general_settings_page);
    
    // 注册通信设置页面
    auto communication_settings_page = std::make_shared<CommunicationSettings>();
    register_page("communication_settings", communication_settings_page);
    
    // 注册内部模板页面
    register_internal_pages();
}

void PageRegistry::register_internal_pages() {
    // 注册错误页面
    auto error_page = std::make_shared<ErrorPage>();
    register_page("__error__", error_page);
    
    // 注册INT设置页面
    auto int_setting_page = std::make_shared<IntSettingPage>();
    register_page("__int_setting__", int_setting_page);
}

} // namespace ui