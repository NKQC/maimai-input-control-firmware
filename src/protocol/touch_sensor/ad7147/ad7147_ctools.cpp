#include "ad7147.h"
#include <algorithm>

// 清空设置到校准所需值
void AD7147::CalibrationTools::Clear_and_prepare_stage_settings(uint8_t stage)
{
    // 清空异常通道bitmap (校准开始时重置)
    pthis->abnormal_channels_bitmap_ = 0;
    
    PortConfig config = pthis->stage_settings_.stages[stage];
    config.afe_offset.bits.pos_afe_offset_swap = 0;
    config.afe_offset.bits.pos_afe_offset = 0;
    config.afe_offset.bits.neg_afe_offset_swap = 0;
    config.afe_offset.bits.neg_afe_offset = 0;
    config.sensitivity.raw = AD7147_SENSITIVITY_DEFAULT;
    config.offset_high = 0;
    config.offset_low = 0;
    config.offset_high_clamp = 0xFFFF;
    config.offset_low_clamp = 0;
    pthis->calirate_save_enabled_channels_mask_ = pthis->enabled_channels_mask_; // 校准时保存的启用的通道掩码
    pthis->enabled_channels_mask_ = ((1 << AD7147_MAX_CHANNELS) - 1);
    pthis->setStageConfig(stage, config);
    pthis->applyEnabledChannelsToHardware();
}

void AD7147::CalibrationTools::Complete_and_restore_calibration()
{
    s1_inited_ = false;
    inited_ = false;
    calibration_state_ = IDLE;
    pthis->enabled_channels_mask_ = pthis->calirate_save_enabled_channels_mask_; // 校准时保存的启用的通道掩码
    pthis->applyEnabledChannelsToHardware();
}

// 设置AFE偏移 0-127
void AD7147::CalibrationTools::Set_AEF_Offset(uint8_t stage, int8_t offset)
{
    PortConfig config = pthis->stage_settings_.stages[stage];
    
    // 限制范围到-127到127
    offset = MAX(-127, MIN(127, offset));
    
    // 自动判定方向并配置偏移
    bool is_positive = offset >= 0;
    uint8_t abs_offset = (uint8_t)(is_positive ? offset : -offset);
    
    config.afe_offset.bits.pos_afe_offset_swap = is_positive ? 0 : (abs_offset > 63 ? 1 : 0);
    config.afe_offset.bits.pos_afe_offset = is_positive ? (abs_offset & 0x3F) : (abs_offset > 63 ? (abs_offset - 64) & 0x3F : 0);
    config.afe_offset.bits.neg_afe_offset_swap = is_positive ? (abs_offset > 63 ? 1 : 0) : 0;
    config.afe_offset.bits.neg_afe_offset = is_positive ? (abs_offset > 63 ? (abs_offset - 64) & 0x3F : 0) : (abs_offset & 0x3F);
    pthis->setStageConfig(stage, config);
}

bool AD7147::CalibrationTools::Read_CDC_Sample(uint8_t stage, CDCSample_result &result)
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
    return result.sample_count++ >= CALIBRATION_SAMPLE_COUNT;
}

bool AD7147::CalibrationTools::Read_Triggle_Sample(uint8_t stage, uint32_t sample, TriggleSample &result)
{
    if (sample & (1u << stage))
        result.triggle_num++;
    else
        result.not_triggle_num++;
    return result.sample_count++ >= CALIBRATION_SAMPLE_COUNT;
}

void AD7147::CalibrationTools::CalibrationLoop(uint32_t sample)
{
    // 首次进入复位硬件配置到校准所需
    if (!inited_)
    {
        inited_ = true;
        has_jump_point = false;
        stage_index_ = 0;
        Clear_and_prepare_stage_settings(stage_index_);
        s1_inited_ = false;
    }

    switch (calibration_state_)
    {
    case IDLE:
    {
        return;
    }

    case PROCESS:
    {
        if (!s1_inited_)
        {
            s1_inited_ = true;
            s1_aef_ = CALIBRATION_STAGE1_SCAN_RANGEA;
            s1_best_aef_ = 0;
            s1_best_ratio_ = 2147483647;
            has_jump_point = false;
            trig_res_.clear();
            // 写入起点AEF
            Set_AEF_Offset(stage_index_, s1_aef_);
            return; // 下次进入开始采样
        }

        // 阶段1每个AEF点进行一轮采样 计算触发比例
        if (!Read_Triggle_Sample(stage_index_, sample, trig_res_)) return; // 继续采样当前AEF点

        // 计算触发比例并更新最佳点
        uint32_t ratio = abs(((trig_res_.triggle_num * 1000) / trig_res_.sample_count) - 500);  // ratio绝对值 我们期望是50% 只需要查看他的偏差
        if (ratio < s1_best_ratio_ && trig_res_.triggle_num)
        {
            s1_best_ratio_ = ratio;
            s1_best_aef_ = s1_aef_;
            if (trig_res_.not_triggle_num) has_jump_point = true;  // 同时存在触发和未触发点位 此时存在跳变点
        }

        // 跳变点的下一步 找到无触发点 随后添加额外的保留偏移
        if (has_jump_point && !trig_res_.triggle_num) {
            // 扫描完成，应用阶段1最佳AEF
            stage_index_++;
            s1_inited_ = false;
            if (stage_index_ >= 12)
            {
                Complete_and_restore_calibration();
            }
            return;
        }

        trig_res_.clear();

        // 推进到下一个AEF点
        #if CALIBRATION_STAGE1_SCAN_RANGEB - CALIBRATION_STAGE1_SCAN_RANGEA
        if (s1_aef_ > CALIBRATION_STAGE1_SCAN_RANGEB)
        #else
        if (s1_aef_ < CALIBRATION_STAGE1_SCAN_RANGEB)
        #endif
        {
            #if CALIBRATION_STAGE1_SCAN_RANGEB - CALIBRATION_STAGE1_SCAN_RANGEA > 0
            s1_aef_++;
            #else
            s1_aef_--;
            #endif
            Set_AEF_Offset(stage_index_, s1_aef_);
            stage_process = MIN((abs(s1_aef_) * 100 / abs(CALIBRATION_STAGE1_SCAN_RANGEA - CALIBRATION_STAGE1_SCAN_RANGEB)), 255);
            return;
        }

        // 扫描结束 能到这里的必然是没找到跳变点 该通道视为异常
        pthis->abnormal_channels_bitmap_ |= (1 << stage_index_);
        stage_index_++;
        s1_inited_ = false;
        if (stage_index_ >= 12)
        {
            Complete_and_restore_calibration();
            return;
        }
        return;
    }
    }
}