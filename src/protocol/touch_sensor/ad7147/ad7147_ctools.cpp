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
    pthis->enabled_channels_mask_ = pthis->calirate_save_enabled_channels_mask_; // 校准时保存的启用的通道掩码
    pthis->applyEnabledChannelsToHardware();
}

// 设置AFE偏移 0-127
void AD7147::CalibrationTools::Set_AEF_Offset(uint8_t stage, Direction direction, uint8_t offset)
{
    PortConfig config = pthis->stage_settings_.stages[stage];
    if (direction == Pos)
    {
        config.afe_offset.bits.pos_afe_offset_swap = 0;
        config.afe_offset.bits.pos_afe_offset = offset & 0x3F;
        config.afe_offset.bits.neg_afe_offset_swap = 0;
        config.afe_offset.bits.neg_afe_offset = 0;
        if (offset > 63)
        {
            config.afe_offset.bits.neg_afe_offset_swap = 1;
            config.afe_offset.bits.neg_afe_offset = (offset - 64) & 0x3F;
        }
    }
    else
    {
        config.afe_offset.bits.neg_afe_offset_swap = 0;
        config.afe_offset.bits.neg_afe_offset = offset & 0x3F;
        config.afe_offset.bits.pos_afe_offset_swap = 0;
        config.afe_offset.bits.pos_afe_offset = 0;
        if (offset > 63)
        {
            config.afe_offset.bits.pos_afe_offset_swap = 1;
            config.afe_offset.bits.pos_afe_offset = (offset - 64) & 0x3F;
        }
    }
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
    // 初始化与跨调用持久状态
    static bool inited = false;
    static uint8_t stage_index = 0; // 当前校准的stage [0..11]

    // 阶段1：扫频寻找震荡点（触发与非触发最接近50%）
    static bool s1_inited = false;
    static int16_t s1_aef = -127;       // 当前扫描AEF -127..127
    static int16_t s1_best_aef = 0;     // 阶段1找到的最佳AEF
    static int8_t s1_best_ratio = 100; // 阶段1最佳触发比例
    static TriggleSample trig_res;      // 阶段1采样累积

    // 阶段2：从阶段1最佳点出发，单步调整AEF使触发占比降至0%
    static bool s2_inited = false;
    static int16_t s2_base_aef = 0;
    static int16_t s2_cur_aef = 0;
    static uint32_t triggle_sample_count = 0; // 触发采样次数
    static uint32_t measure_time_tag = 0;

    // 同步当前stage索引（供外部观察）
    current_stage_index_ = stage_index;

    // 首次进入复位硬件配置到校准所需
    if (!inited)
    {
        inited = true;
        Clear_and_prepare_stage_settings(stage_index);
        s1_inited = false;
        s2_inited = false;
    }

    switch (calibration_state_)
    {
    case IDLE:
    {
        return;
    }

    case Stage1_baseline:
    {
        if (!s1_inited)
        {
            s1_inited = true;
            s1_aef = -127;
            s1_best_aef = 0;
            s1_best_ratio = 100;
            trig_res.clear();
            // 写入起点AEF
            Direction dir = (s1_aef >= 0) ? Pos : Neg;
            uint8_t off = (uint8_t)((s1_aef >= 0) ? s1_aef : -s1_aef);
            Set_AEF_Offset(stage_index, dir, off);
            return; // 下次进入开始采样
        }

        // 阶段1每个AEF点进行一轮采样（50次，见CALIBRATION_SAMPLE_COUNT），计算触发比例
        if (!Read_Triggle_Sample(stage_index, sample, trig_res)) return; // 继续采样当前AEF点

        // 计算触发比例并更新最佳点
        int8_t ratio = abs((trig_res.triggle_num * 100) / trig_res.sample_count - 50);  // ratio绝对值 我们期望是50% 只需要查看他的偏差
        if (ratio < s1_best_ratio)
        {
            s1_best_ratio = ratio;
            s1_best_aef = s1_aef;
        }
        trig_res.clear();

        // 推进到下一个AEF点
        if (s1_aef < 127)
        {
            s1_aef++;
            Direction dir = (s1_aef >= 0) ? Pos : Neg;
            uint8_t off = (uint8_t)((s1_aef >= 0) ? s1_aef : -s1_aef);
            Set_AEF_Offset(stage_index, dir, off);
            stage_process = (s1_aef + 127) / 2;
            return;
        }

        // 扫描完成，应用阶段1最佳AEF
        {
            Direction best_dir = (s1_best_aef >= 0) ? Pos : Neg;
            uint8_t best_off = (uint8_t)((s1_best_aef >= 0) ? s1_best_aef : -s1_best_aef);
            Set_AEF_Offset(stage_index, best_dir, best_off);
        }

        // 切换到阶段2
        s2_inited = false;
        calibration_state_ = Stage2_Offset_calibration;
        return;
    }

    case Stage2_Offset_calibration:
    {
        if (!s2_inited)
        {
            s2_inited = true;
            s2_base_aef = s1_best_aef;
            s2_cur_aef = s2_base_aef;
            measure_time_tag = us_to_ms(time_us_32()) + STAGE2_MEASURE_TIME_MS;
            return;
        }

        // 每步采样
        if (measure_time_tag > us_to_ms(time_us_32()))
        {
            Read_Triggle_Sample(stage_index, sample, trig_res);
            triggle_sample_count += trig_res.triggle_num;
            trig_res.clear();
            return;
        }

        // 完成N次采样，评估结果
        if (!triggle_sample_count)
        {
            // 当前stage达成目标，进入下一个stage
            stage_index++;
            if (stage_index >= 12)
            {
                Complete_and_restore_calibration();
                current_stage_index_ = 12;
                calibration_state_ = IDLE;
                inited = false; // 为下次校准流程复位
                return;
            }
            else
            {
                current_stage_index_ = stage_index;
                calibration_state_ = Stage1_baseline;
                s1_inited = false;
                s2_inited = false;
                return;
            }
        }

        // 按当前方向单步调整
        s2_cur_aef--;

        // 限制最大调整范围 -48
        if (s2_cur_aef < s2_base_aef - 48)
        {
            // 超出范围，放弃当前stage，进入下一stage
            stage_index++;
            if (stage_index >= 12)
            {
                current_stage_index_ = 12;
                calibration_state_ = IDLE;
                inited = false;
                pthis->abnormal_channels_bitmap_ |= (1 << (stage_index - 1));
                return;
            }
            else
            {
                current_stage_index_ = stage_index;
                Clear_and_prepare_stage_settings(stage_index);
                calibration_state_ = Stage1_baseline;
                s1_inited = false;
                s2_inited = false;
                return;
            }
        }

        // 应用新的AEF并开始下一轮5次采样
        {
            Direction dir = (s2_cur_aef >= 0) ? Pos : Neg;
            uint8_t off = (uint8_t)((s2_cur_aef >= 0) ? s2_cur_aef : -s2_cur_aef);
            Set_AEF_Offset(stage_index, dir, off);
        }
        stage_process = stage_process > 252 ? 252 : stage_process + 1;
        measure_time_tag = 0;
        triggle_sample_count = 0;
        measure_time_tag = us_to_ms(time_us_32()) + STAGE2_MEASURE_TIME_MS;
        return;
    }
    }
}