#include "ad7147.h"
#include <algorithm>
#include "src/protocol/usb_serial_logs/usb_serial_logs.h"

// 清空设置到校准所需值 - 一次性准备全部通道
void AD7147::CalibrationTools::Clear_and_prepare_stage_settings()
{
    // 一次性初始化全局设置和所有通道
    pthis->abnormal_channels_bitmap_ = 0;
    pthis->calirate_save_enabled_channels_mask_ = pthis->enabled_channels_mask_; // 校准时保存的启用的通道掩码
    pthis->enabled_channels_mask_ = ((1 << AD7147_MAX_CHANNELS) - 1);
    pthis->applyEnabledChannelsToHardware();
    
    // 一次性准备所有通道的配置和初始化状态
    for (uint8_t ch = 0; ch < AD7147_MAX_CHANNELS; ch++) {
        // 初始化通道校准状态
        s1_inited_[ch] = true;
        s1_aef_[ch] = CALIBRATION_STAGE1_SCAN_RANGEA;
        s1_best_aef_[ch] = 0;
        cdc_samples_[ch].clear();
        trigger_samples_[ch].clear();
        max_fluctuation_[ch] = 0;
        // 写入起点AEF
        Set_AEF_Offset(ch, s1_aef_[ch]);
    }
    
    global_initialized_ = true;
}

void AD7147::CalibrationTools::Complete_and_restore_calibration()
{
    inited_ = false;
    calibration_state_ = IDLE;
    pthis->enabled_channels_mask_ = pthis->calirate_save_enabled_channels_mask_; // 校准时保存的启用的通道掩码
    pthis->applyEnabledChannelsToHardware();
    
    // 重置全局初始化标志，以便下次校准时重新初始化
    global_initialized_ = false;
}

// 设置AFE偏移 -127 ~ 127
void AD7147::CalibrationTools::Set_AEF_Offset(uint8_t stage, int16_t offset)
{
    PortConfig config = pthis->stage_settings_.stages[stage];
    
    // 自动判定方向并配置偏移
    bool is_positive = offset >= 0;
    uint16_t abs_offset = abs(MAX(-127, MIN(127, offset)));
    
    config.afe_offset.bits.pos_afe_offset_swap = !is_positive;
    config.afe_offset.bits.neg_afe_offset_swap = is_positive;
    config.afe_offset.bits.pos_afe_offset = abs_offset > 63 ? 63 : abs_offset;
    config.afe_offset.bits.neg_afe_offset = MAX(abs_offset - 63, 0);
    pthis->setStageConfig(stage, config);
}

bool AD7147::CalibrationTools::Read_CDC_Sample(uint8_t stage, CDCSample_result &result, bool measure)
{
    static uint16_t value;
    pthis->readStageCDC_direct(stage, value);
    if (result.sample_count)
    {
        result.average = (uint16_t)((result.average * result.sample_count + value) / (result.sample_count + 1));
    }
    else
        result.average = value;
    if (!result.min) result.min = result.average;
    result.max = MAX(result.max, value);
    result.min = MIN(result.min, value);
    return (result.sample_count++ >= (measure ? CALIBRATION_MEASURE_SAMPLE_COUNT : CALIBRATION_SCAN_SAMPLE_COUNT));
}

bool AD7147::CalibrationTools::Read_Triggle_Sample(uint8_t stage, uint32_t sample, TriggleSample &result, bool measure)
{
    if (sample & (1u << stage)) {
        result.triggle_num++;
        if (measure) return true;
    }
    else
        result.not_triggle_num++;
    return (result.sample_count++ >= (measure ? CALIBRATION_MEASURE_SAMPLE_COUNT : CALIBRATION_SCAN_SAMPLE_COUNT));
}

void AD7147::CalibrationTools::CalibrationLoop(uint32_t sample)
{
    // 首次进入复位硬件配置到校准所需
    if (!inited_)
    {
        inited_ = true;
        // 一次性准备所有通道的校准设置和初始化状态
        Clear_and_prepare_stage_settings();
    }

    switch (calibration_state_)
    {
    case IDLE:
    {
        return;
    }

    case PROCESS:
    {
        bool all_channels_completed = true;
        uint32_t total_progress = 0;
        uint16_t target_value = AD7147_CALIBRATION_TARGET_VALUE;

        // 同时处理所有12个通道
        for (uint8_t stage = 0; stage < AD7147_MAX_CHANNELS; stage++)
        {
            // 跳过已完成或异常的通道
            if (!s1_inited_[stage]) {
                total_progress += 255;
                continue;
            }
            all_channels_completed = false;
            
            // 进行CDC采样
            if (!Read_CDC_Sample(stage, cdc_samples_[stage], false)) continue; // 继续采样当前AEF点

            // 计算当前波动差并更新最大波动差
            uint16_t current_fluctuation = abs(cdc_samples_[stage].max - cdc_samples_[stage].min);
            if (current_fluctuation > max_fluctuation_[stage]) {
                max_fluctuation_[stage] = current_fluctuation;
            }
            // 验证当前CDC是否高于目标值 + 基于正向指数算法的波动调整
            // 正向指数算法: 波动值越大给越大调整，通过噪声判断灵敏度需求
            uint16_t fluctuation_factor = 0;
            if (max_fluctuation_[stage] <= FLUCTUATION_MIN_THRESHOLD) {
                // 波动很小，给最小调整值
                fluctuation_factor = max_fluctuation_[stage] / FLUCTUATION_MIN_FACTOR; // 最小调整系数
            } else if (max_fluctuation_[stage] >= FLUCTUATION_MAX_THRESHOLD) {
                // 波动很大，给最大调整值
                fluctuation_factor = max_fluctuation_[stage] * FLUCTUATION_MAX_FACTOR; // 最大调整系数
            } else {
                // 中间区域使用正向指数函数: factor = min_val + (max_val - min_val) * (1 - exp(-k * (x - 50) / (10000 - 50)))
                // 波动越大，调整值越大，实现噪声自适应灵敏度
                uint32_t x_normalized = max_fluctuation_[stage] - FLUCTUATION_MIN_THRESHOLD; // 减去最小阈值
                
                // 使用泰勒级数近似指数函数: e^(-t) ≈ 1 - t + t²/2 - t³/6 + ...
                // 其中 t = k * x_normalized / TAYLOR_NORMALIZATION_RANGE, k由sensitivity_target控制
                uint32_t k_factor = (TAYLOR_SCALE_FACTOR * sensitivity_target) / TAYLOR_K_DIVISOR; // sensitivity_target越大，k越大，增长越快
                uint32_t t = (k_factor * x_normalized) / TAYLOR_NORMALIZATION_RANGE; // 缩放到合适范围
                
                // 泰勒级数前4项计算 e^(-t) * TAYLOR_SCALE_FACTOR
                uint32_t exp_neg_t = TAYLOR_SCALE_FACTOR; // 初始值 1 * TAYLOR_SCALE_FACTOR
                if (t < TAYLOR_SCALE_FACTOR) { // 避免溢出
                    exp_neg_t = exp_neg_t - t; // -t项
                    if (t < TAYLOR_SCALE_FACTOR / 2) {
                        exp_neg_t = exp_neg_t + (t * t) / (2 * TAYLOR_SCALE_FACTOR); // +t²/2项
                        if (t < TAYLOR_SCALE_FACTOR / 4) {
                            exp_neg_t = exp_neg_t - (t * t * t) / (6 * TAYLOR_SCALE_FACTOR * TAYLOR_SCALE_FACTOR); // -t³/6项
                        }
                    }
                } else {
                    exp_neg_t = 0; // 当t很大时，e^(-t)接近0
                }
                
                // 计算正向指数因子: 从0.1倍到2倍的指数增长
                uint32_t min_factor = max_fluctuation_[stage] / FLUCTUATION_MIN_FACTOR; // 最小调整值(低噪声时)
                uint32_t max_factor = max_fluctuation_[stage] * FLUCTUATION_MAX_FACTOR; // 最大调整值(高噪声时)
                uint32_t growth_factor = TAYLOR_SCALE_FACTOR - exp_neg_t; // 1 - e^(-t)
                fluctuation_factor = min_factor + ((max_factor - min_factor) * growth_factor) / TAYLOR_SCALE_FACTOR;
            }
            uint16_t adjusted_target = target_value + fluctuation_factor;
            if (cdc_samples_[stage].average >= adjusted_target) {
                // 达到目标CDC值，开始检查触发状态
                if (!Read_Triggle_Sample(stage, sample, trigger_samples_[stage], true)) continue; // 继续采样触发状态
                USB_LOG_DEBUG("On Target CDC: stage: %d, cdc: %d, triggle: %d, not_triggle: %d", stage, cdc_samples_[stage].average, trigger_samples_[stage].triggle_num, trigger_samples_[stage].not_triggle_num);
                // 检查是否仍然触发
                if (!trigger_samples_[stage].triggle_num) {
                    // 不再触发，该通道完成校准
                    Set_AEF_Offset(stage, s1_best_aef_[stage] + CALIBRATION_AEF_SAVE_AREA);
                    s1_inited_[stage] = false;
                    continue;
                }
                trigger_samples_[stage].clear();
            }

            // 记录当前最佳AEF
            s1_best_aef_[stage] = s1_aef_[stage];
            cdc_samples_[stage].clear();

            // 推进到下一个AEF点
            #if (CALIBRATION_STAGE1_SCAN_RANGEB - CALIBRATION_STAGE1_SCAN_RANGEA) < 0
            if (s1_aef_[stage] > CALIBRATION_STAGE1_SCAN_RANGEB)
            #else
            if (s1_aef_[stage] < CALIBRATION_STAGE1_SCAN_RANGEB)
            #endif
            {
                #if (CALIBRATION_STAGE1_SCAN_RANGEB - CALIBRATION_STAGE1_SCAN_RANGEA) < 0
                s1_aef_[stage]--;
                #else
                s1_aef_[stage]++;
                #endif
                Set_AEF_Offset(stage, s1_aef_[stage]);
                
                // 计算当前通道进度并累加到总进度 (范围0-255)
                total_progress += MIN(((abs(s1_aef_[stage] - CALIBRATION_STAGE1_SCAN_RANGEA) * 255) / abs(CALIBRATION_STAGE1_SCAN_RANGEA - CALIBRATION_STAGE1_SCAN_RANGEB)), 255);
                continue;
            }
            // 扫描结束，调整到头也低于目标值，该通道视为异常
            pthis->abnormal_channels_bitmap_ |= (1 << stage);
            s1_inited_[stage] = false; // 标记为已完成（异常）
        }

        // 计算总进度为所有通道的平均进度 (范围0-255)
        if (!all_channels_completed) {
            stage_process = MIN((total_progress / AD7147_MAX_CHANNELS), 255);
        } else {
            stage_process = 255; // 所有通道都已完成
            Complete_and_restore_calibration();
        }

        return;
    }
    }
}