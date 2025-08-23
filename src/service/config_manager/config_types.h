#pragma once

#include <map>
#include <string>
#include <cstdint>
#include <limits>

// 配置值类型枚举
enum class ConfigValueType {
  BOOL,
  INT8,
  UINT8,
  UINT16,
  UINT32,
  FLOAT,
  STRING
};

// 配置值结构体
struct ConfigValue {
  ConfigValueType type;
  union {
    bool bool_val;
    int8_t int8_val;
    uint8_t uint8_val;
    uint16_t uint16_val;
    uint32_t uint32_val;
    float float_val;
  };
  std::string string_val;
  
  // 范围限制
  union {
    int8_t int8_min;
    uint8_t uint8_min;
    uint16_t uint16_min;
    uint32_t uint32_min;
    float float_min;
  } min_val;
  
  union {
    int8_t int8_max;
    uint8_t uint8_max;
    uint16_t uint16_max;
    uint32_t uint32_max;
    float float_max;
  } max_val;
  
  bool has_range;
  
  ConfigValue() : type(ConfigValueType::BOOL), bool_val(false), has_range(false) {}
  ConfigValue(bool val) : type(ConfigValueType::BOOL), bool_val(val), has_range(false) {}
  
  ConfigValue(int8_t val, int8_t min_v = std::numeric_limits<int8_t>::min(), int8_t max_v = std::numeric_limits<int8_t>::max()) 
    : type(ConfigValueType::INT8), int8_val(val), has_range(min_v != std::numeric_limits<int8_t>::min() || max_v != std::numeric_limits<int8_t>::max()) {
    min_val.int8_min = min_v;
    max_val.int8_max = max_v;
    clamp_value();
  }
  
  ConfigValue(uint8_t val, uint8_t min_v = std::numeric_limits<uint8_t>::min(), uint8_t max_v = std::numeric_limits<uint8_t>::max()) 
    : type(ConfigValueType::UINT8), uint8_val(val), has_range(min_v != std::numeric_limits<uint8_t>::min() || max_v != std::numeric_limits<uint8_t>::max()) {
    min_val.uint8_min = min_v;
    max_val.uint8_max = max_v;
    clamp_value();
  }
  
  ConfigValue(uint16_t val, uint16_t min_v = std::numeric_limits<uint16_t>::min(), uint16_t max_v = std::numeric_limits<uint16_t>::max()) 
    : type(ConfigValueType::UINT16), uint16_val(val), has_range(min_v != std::numeric_limits<uint16_t>::min() || max_v != std::numeric_limits<uint16_t>::max()) {
    min_val.uint16_min = min_v;
    max_val.uint16_max = max_v;
    clamp_value();
  }
  
  ConfigValue(uint32_t val, uint32_t min_v = std::numeric_limits<uint32_t>::min(), uint32_t max_v = std::numeric_limits<uint32_t>::max()) 
    : type(ConfigValueType::UINT32), uint32_val(val), has_range(min_v != std::numeric_limits<uint32_t>::min() || max_v != std::numeric_limits<uint32_t>::max()) {
    min_val.uint32_min = min_v;
    max_val.uint32_max = max_v;
    clamp_value();
  }
  
  ConfigValue(float val, float min_v = std::numeric_limits<float>::lowest(), float max_v = std::numeric_limits<float>::max()) 
    : type(ConfigValueType::FLOAT), float_val(val), has_range(min_v != std::numeric_limits<float>::lowest() || max_v != std::numeric_limits<float>::max()) {
    min_val.float_min = min_v;
    max_val.float_max = max_v;
    clamp_value();
  }
  
  ConfigValue(const std::string& val) : type(ConfigValueType::STRING), string_val(val), has_range(false) {}
  ConfigValue(const char* val) : type(ConfigValueType::STRING), string_val(val), has_range(false) {}
  
  // 限制数值到指定范围内
  void clamp_value() {
    if (!has_range) return;
    
    switch (type) {
      case ConfigValueType::INT8:
        if (int8_val < min_val.int8_min) int8_val = min_val.int8_min;
        if (int8_val > max_val.int8_max) int8_val = max_val.int8_max;
        break;
      case ConfigValueType::UINT8:
        if (uint8_val < min_val.uint8_min) uint8_val = min_val.uint8_min;
        if (uint8_val > max_val.uint8_max) uint8_val = max_val.uint8_max;
        break;
      case ConfigValueType::UINT16:
        if (uint16_val < min_val.uint16_min) uint16_val = min_val.uint16_min;
        if (uint16_val > max_val.uint16_max) uint16_val = max_val.uint16_max;
        break;
      case ConfigValueType::UINT32:
        if (uint32_val < min_val.uint32_min) uint32_val = min_val.uint32_min;
        if (uint32_val > max_val.uint32_max) uint32_val = max_val.uint32_max;
        break;
      case ConfigValueType::FLOAT:
        if (float_val < min_val.float_min) float_val = min_val.float_min;
        if (float_val > max_val.float_max) float_val = max_val.float_max;
        break;
      default:
        break;
    }
  }
  
  // 设置新值并自动限制范围
  void set_value(int8_t val) {
    if (type == ConfigValueType::INT8) {
      int8_val = val;
      clamp_value();
    }
  }
  
  void set_value(uint8_t val) {
    if (type == ConfigValueType::UINT8) {
      uint8_val = val;
      clamp_value();
    }
  }
  
  void set_value(uint16_t val) {
    if (type == ConfigValueType::UINT16) {
      uint16_val = val;
      clamp_value();
    }
  }
  
  void set_value(uint32_t val) {
    if (type == ConfigValueType::UINT32) {
      uint32_val = val;
      clamp_value();
    }
  }
  
  void set_value(float val) {
    if (type == ConfigValueType::FLOAT) {
      float_val = val;
      clamp_value();
    }
  }
  
  // 从另一个ConfigValue复制范围限制信息
  void copy_range_from(const ConfigValue& other) {
    if (type == other.type) {
      has_range = other.has_range;
      if (has_range) {
        min_val = other.min_val;
        max_val = other.max_val;
      }
    }
  }
};

// 配置映射类型定义
typedef std::map<std::string, ConfigValue> config_map_t;
