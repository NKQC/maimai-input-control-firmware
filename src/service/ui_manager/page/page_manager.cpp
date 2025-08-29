#include "page_manager.h"
#include "page_types.h"
#include "main_menu_page.h"
#include "settings_page.h"
#include "status_page.h"

namespace ui {

PageManager::PageManager() : graphics_engine_(nullptr) {
}

PageManager::~PageManager() {
    deinit();
}

bool PageManager::init(GraphicsEngine* graphics_engine) {
    graphics_engine_ = graphics_engine;
    
    if (!graphics_engine_) {
        return false;
    }
    
    // 创建并注册所有页面
    register_page("main_menu", create_main_menu_page());
    register_page("settings", create_settings_page());
    register_page("status", create_status_page());
    register_page("calibration", create_calibration_page());
    register_page("diagnostics", create_diagnostics_page());
    
    return true;
}

void PageManager::deinit() {
    pages_.clear();
}

std::shared_ptr<PageTemplate> PageManager::get_page(const std::string& page_id) const {
    auto it = pages_.find(page_id);
    if (it != pages_.end()) {
        return it->second;
    }
    return nullptr;
}

void PageManager::register_page(const std::string& page_id, std::shared_ptr<PageTemplate> page) {
    if (page) {
        pages_[page_id] = page;
    }
}

void PageManager::unregister_page(const std::string& page_id) {
    pages_.erase(page_id);
}

bool PageManager::has_page(const std::string& page_id) const {
    return pages_.find(page_id) != pages_.end();
}

std::shared_ptr<PageTemplate> PageManager::create_main_menu_page() {
    auto page = std::make_shared<MainMenuPage>(graphics_engine_);
    return page;
}

std::shared_ptr<PageTemplate> PageManager::create_settings_page() {
    auto page = std::make_shared<SettingsPage>(graphics_engine_);
    return page;
}

std::shared_ptr<PageTemplate> PageManager::create_status_page() {
    auto page = std::make_shared<StatusPage>(graphics_engine_);
    return page;
}

std::shared_ptr<PageTemplate> PageManager::create_calibration_page() {
    auto page = std::make_shared<PageTemplate>(graphics_engine_);
    return page;
}

std::shared_ptr<PageTemplate> PageManager::create_diagnostics_page() {
    auto page = std::make_shared<PageTemplate>(graphics_engine_);
    return page;
}

} // namespace ui