#include "page_constructor.h"

namespace ui {

// 静态成员定义
std::map<std::string, std::string> PageConstructor::shared_data_map_;

void PageConstructor::set_shared_data(const std::string& key, const std::string& value) {
    shared_data_map_[key] = value;
}

std::string PageConstructor::get_shared_data(const std::string& key, const std::string& default_value) {
    auto it = shared_data_map_.find(key);
    if (it != shared_data_map_.end()) {
        return it->second;
    }
    return default_value;
}

bool PageConstructor::remove_shared_data(const std::string& key) {
    auto it = shared_data_map_.find(key);
    if (it != shared_data_map_.end()) {
        shared_data_map_.erase(it);
        return true;
    }
    return false;
}

bool PageConstructor::has_shared_data(const std::string& key) {
    return shared_data_map_.find(key) != shared_data_map_.end();
}

void PageConstructor::clear_shared_data() {
    shared_data_map_.clear();
}

size_t PageConstructor::get_shared_data_count() {
    return shared_data_map_.size();
}

} // namespace ui