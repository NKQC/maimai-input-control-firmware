#include "ad7147.h"
#include <algorithm>

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
            uint16_t current_fluctuation = cdc_samples_[stage].max - cdc_samples_[stage].min;
            if (current_fluctuation > max_fluctuation_[stage]) {
                max_fluctuation_[stage] = current_fluctuation;
            }

            // 验证当前CDC是否高于目标值 + 最大波动差的绝对值
            uint16_t adjusted_target = target_value + (max_fluctuation_[stage] / 3);
            if (cdc_samples_[stage].average >= adjusted_target) {
                // 达到目标CDC值，开始检查触发状态
                if (!Read_Triggle_Sample(stage, sample, trigger_samples_[stage], false)) continue; // 继续采样触发状态
                
                // 检查是否仍然触发
                if (trigger_samples_[stage].triggle_num > 0) {
                    // 仍然触发，清理触发采样数据并继续推进AFE
                    trigger_samples_[stage].clear();
                } else {
                    // 不再触发，该通道完成校准
                    Set_AEF_Offset(stage, s1_best_aef_[stage] + CALIBRATION_AEF_SAVE_AREA);
                    s1_inited_[stage] = false;
                    continue;
                }
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