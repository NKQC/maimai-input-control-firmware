#ifndef SELECTOR_TEST_H
#define SELECTOR_TEST_H

#include "../engine/page_construction/page_template.h"
#include "../engine/page_construction/page_constructor.h"

namespace ui {

/**
 * 选择器功能测试页面
 * 用于测试选择器的锁定、解锁和回调功能
 */
class SelectorTest : public PageConstructor {
public:
    SelectorTest();
    
    /**
     * 渲染测试页面
     * @param page_template 页面模板引用
     */
    void render(PageTemplate& page_template) override;
    
    /**
     * 获取测试值
     * @return 当前测试值
     */
    int get_test_value() const;
    
    /**
     * 设置测试值
     * @param value 新的测试值
     */
    void set_test_value(int value);
    
private:
    int test_value_;  // 测试用的数值
};

} // namespace ui

#endif // SELECTOR_TEST_H