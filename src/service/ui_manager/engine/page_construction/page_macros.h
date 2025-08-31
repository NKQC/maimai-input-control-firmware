#ifndef PAGE_MACROS_H
#define PAGE_MACROS_H

#include "page_template.h"
#include <vector>

/**
 * 页面构造宏定义
 * 简化页面代码编写，提供统一的页面构造接口
 */

// 页面开始宏 - 清空页面并初始化行容器
#define PAGE_START() \
    page_template.flush(); \
    std::vector<LineConfig> all_lines;

// 添加行宏 - 向页面添加一行内容
#define ADD_LINE(line_config) \
    all_lines.push_back(line_config);

// 添加文本行宏 - 快速添加文本行
#define ADD_TEXT(text, color, align) \
    all_lines.push_back(LineConfig(text, color, static_cast<LineAlign>(align)));

// 添加状态行宏 - 快速添加状态行
#define ADD_STATUS(text, color, align) \
    all_lines.push_back(LineConfig::create_status_line(text, color, static_cast<LineAlign>(align)));

// 添加菜单项宏 - 快速添加菜单跳转项
#define ADD_MENU(text, target_page, color) \
    all_lines.push_back(LineConfig::create_menu_jump(text, target_page, color));

// 添加进度条宏 - 快速添加进度条
#define ADD_PROGRESS(progress_ptr, color) \
    all_lines.push_back(LineConfig::create_progress_bar(progress_ptr, color));

// 添加整数设置宏 - 快速添加整数设置项
#define ADD_INT_SETTING(value_ptr, min_val, max_val, display_text, title, change_cb, complete_cb, color) \
    all_lines.push_back(LineConfig::create_int_setting(value_ptr, min_val, max_val, display_text, title, change_cb, complete_cb, color));

// 添加按钮宏 - 快速添加按钮项
#define ADD_BUTTON(text, callback, color, align) \
    all_lines.push_back(LineConfig::create_button(text, callback, color, static_cast<LineAlign>(align)));

// 添加返回项宏 - 快速添加返回项
#define ADD_BACK_ITEM(text, color) \
    all_lines.push_back(LineConfig::create_back_item(text, color));

// 添加选择器宏 - 快速添加选择器项
#define ADD_SELECTOR(text, selector_cb, lock_cb, color, align) \
    all_lines.push_back(LineConfig::create_selector(text, selector_cb, lock_cb, color, static_cast<LineAlign>(align)));

// 添加简单选择器宏 - 快速添加选择器项（无锁定回调）
#define ADD_SIMPLE_SELECTOR(text, selector_cb, color) \
    all_lines.push_back(LineConfig::create_selector(text, selector_cb, nullptr, color, LineAlign::LEFT));

// 页面结束宏 - 设置所有行并启用滚动
#define PAGE_END() \
    page_template.set_all_lines(all_lines);

// 页面绘制宏 - 绘制页面
#define PAGE_DRAW() \
    page_template.draw();

// 页面跳过宏 - 跳过当前页面处理
#define PAGE_SKIP() \
    return;

// 设置标题宏 - 快速设置页面标题
#define SET_TITLE(title, color) \
    page_template.set_title(title, color);

// 组合宏 - 完整的页面构造流程
#define PAGE_WITH_TITLE(title, color) \
    PAGE_START() \
    SET_TITLE(title, color)

// 简化的文本页面宏
#define SIMPLE_TEXT_PAGE(title, lines_vector) \
    PAGE_START() \
    SET_TITLE(title, COLOR_WHITE) \
    for (const auto& line : lines_vector) { \
        ADD_LINE(line); \
    } \
    PAGE_END()

// 简化的菜单页面宏
#define SIMPLE_MENU_PAGE(title, menu_items) \
    PAGE_START() \
    SET_TITLE(title, COLOR_WHITE) \
    for (const auto& item : menu_items) { \
        ADD_MENU(item.first, item.second, COLOR_TEXT_WHITE); \
    } \
    PAGE_END()

#endif // PAGE_MACROS_H