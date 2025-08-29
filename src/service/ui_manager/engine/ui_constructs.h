#ifndef UI_CONSTRUCTS_H
#define UI_CONSTRUCTS_H

#include "../page/page_template.h"
#include "../page/page_types.h"
#include <functional>
#include <string>
#include <vector>
#include <memory>
#include <map>

// 前向声明
class UIManager;

// 动态字符串生成函数类型
using DynamicStringFunc = std::function<std::string()>;

// 构造体基类
class UIConstruct {
public:
    virtual ~UIConstruct() = default;
    
    // 获取显示文本（支持动态生成）
    virtual std::string get_display_text() const = 0;
    
    // 处理交互事件
    virtual bool handle_interaction() = 0;
    
    // 检查是否为交互元素
    virtual bool is_interactive() const = 0;
    
    // 获取构造体类型
    virtual std::string get_type() const = 0;
    
    // 更新动态内容（每帧调用）
    virtual void update() {}
    
    // 设置选中状态
    virtual void set_selected(bool selected) { selected_ = selected; }
    virtual bool is_selected() const { return selected_; }
    
protected:
    bool selected_ = false;
};

// 页面跳转构造体
class PageJumpConstruct : public UIConstruct {
public:
    // 构造函数 - 静态文本
    PageJumpConstruct(const std::string& text, ui::UIPage target_page);
    
    // 动态文本构造函数
    PageJumpConstruct(DynamicStringFunc text_func, ui::UIPage target_page);
    
    // UIConstruct接口实现
    std::string get_display_text() const override;
    bool handle_interaction() override;
    bool is_interactive() const override { return true; }
    std::string get_type() const override { return "PageJump"; }
    void update() override;
    
    // 获取目标页面
    ui::UIPage get_target_page() const { return target_page_; }
    
private:
    std::string static_text_;
    DynamicStringFunc dynamic_text_func_;
    ui::UIPage target_page_;
    bool use_dynamic_text_;
};

// 按钮构造体
class ButtonConstruct : public UIConstruct {
public:
    using ButtonCallback = std::function<void()>;
    
    // 构造函数 - 静态文本
    ButtonConstruct(const std::string& text, ButtonCallback callback);
    
    // 构造函数 - 动态文本
    ButtonConstruct(DynamicStringFunc text_func, ButtonCallback callback);
    
    // UIConstruct接口实现
    std::string get_display_text() const override;
    bool handle_interaction() override;
    bool is_interactive() const override { return true; }
    std::string get_type() const override { return "Button"; }
    void update() override;
    
private:
    std::string static_text_;
    DynamicStringFunc dynamic_text_func_;
    ButtonCallback callback_;
    bool use_dynamic_text_;
};

// 设置构造体
class SettingsConstruct : public UIConstruct {
public:
    using SettingsCallback = std::function<void(int value)>;
    
    // 构造函数 - 静态文本
    SettingsConstruct(const std::string& text, int* target_variable, 
                     int min_value, int max_value, SettingsCallback callback = nullptr);
    
    // 构造函数 - 动态文本
    SettingsConstruct(DynamicStringFunc text_func, int* target_variable,
                     int min_value, int max_value, SettingsCallback callback = nullptr);
    
    // UIConstruct接口实现
    std::string get_display_text() const override;
    bool handle_interaction() override;
    bool is_interactive() const override { return true; }
    std::string get_type() const override { return "Settings"; }
    void update() override;
    
    // 设置相关方法
    void adjust_value(int delta);
    int get_current_value() const;
    void set_value(int value);
    int get_min_value() const { return min_value_; }
    int get_max_value() const { return max_value_; }
    
private:
    std::string static_text_;
    DynamicStringFunc dynamic_text_func_;
    int* target_variable_;
    int min_value_;
    int max_value_;
    SettingsCallback callback_;
    bool use_dynamic_text_;
    
    void clamp_value();
};

// 文本显示构造体（非交互）
class TextConstruct : public UIConstruct {
public:
    // 构造函数 - 静态文本
    explicit TextConstruct(const std::string& text);
    
    // 构造函数 - 动态文本
    explicit TextConstruct(DynamicStringFunc text_func);
    
    // UIConstruct接口实现
    std::string get_display_text() const override;
    bool handle_interaction() override { return false; }
    bool is_interactive() const override { return false; }
    std::string get_type() const override { return "Text"; }
    void update() override;
    
private:
    std::string static_text_;
    DynamicStringFunc dynamic_text_func_;
    bool use_dynamic_text_;
};

// 构造体页面管理器
class ConstructPage {
public:
    ConstructPage(const std::string& title = "");
    ~ConstructPage();
    
    // 添加构造体
    void add_construct(std::shared_ptr<UIConstruct> construct);
    
    // 移除构造体
    void remove_construct(size_t index);
    void clear_constructs();
    
    // 获取构造体
    std::shared_ptr<UIConstruct> get_construct(size_t index) const;
    size_t get_construct_count() const;
    
    // 页面属性
    void set_title(const std::string& title) { title_ = title; }
    std::string get_title() const { return title_; }
    
    // 交互管理
    bool has_interactive_elements() const;
    std::vector<size_t> get_interactive_indices() const;
    
    // 导航管理
    void set_selected_index(int index);
    int get_selected_index() const { return selected_index_; }
    bool navigate_up();
    bool navigate_down();
    bool navigate_left();
    bool navigate_right();
    bool handle_confirm();
    
    // 更新所有构造体
    void update_all();
    
    // 渲染到PageTemplate
    void render_to_page_template(PageTemplate& page_template);
    
private:
    std::string title_;
    std::vector<std::shared_ptr<UIConstruct>> constructs_;
    int selected_index_;
    
    void update_selection();
    int find_next_interactive(int start_index, bool forward) const;
};

// 页面回退管理器
class PageNavigationManager {
public:
    static PageNavigationManager& getInstance();
    
    // 页面栈管理
    void push_page(ui::UIPage page);
    ui::UIPage pop_page();
    ui::UIPage get_current_page() const;
    ui::UIPage get_previous_page() const;
    bool can_go_back() const;
    
    // 返回导航处理
    ui::UIPage handle_back_navigation(bool has_interactive_content);
    
    // 设置主页面
    void set_main_page(ui::UIPage main_page) { main_page_ = main_page; }
    ui::UIPage get_main_page() const { return main_page_; }
    
    // 清空页面栈
    void clear_stack();
    
private:
    PageNavigationManager() = default;
    std::vector<ui::UIPage> page_stack_;
    ui::UIPage main_page_ = ui::UIPage::MAIN;
};

// 菜单交互系统
class MenuInteractionSystem {
public:
    static MenuInteractionSystem& getInstance();
    
    // 注册构造体页面
    void register_page(ui::UIPage page_id, std::shared_ptr<PageTemplate> page);
    void unregister_page(ui::UIPage page_id);
    
    // 页面获取
    std::shared_ptr<PageTemplate> get_current_page() const;
    std::shared_ptr<PageTemplate> get_page(ui::UIPage page_id) const;
    
    // 页面切换
    bool switch_to_page(ui::UIPage page_id);
    
    // 输入处理
    bool handle_joystick_up();
    bool handle_joystick_down();
    bool handle_joystick_left();
    bool handle_joystick_right();
    bool handle_joystick_confirm();
    bool handle_back_button();
    
    // 更新系统
    void update();
    
    // 渲染当前页面
    void render_current_page(PageTemplate& page_template);
    
private:
    MenuInteractionSystem() = default;
    std::map<ui::UIPage, std::shared_ptr<PageTemplate>> pages_;
    ui::UIPage current_page_id_ = ui::UIPage::MAIN;
};

#endif // UI_CONSTRUCTS_H