#include "ad7147.h"

// 清空设置到校准所需值
void AD7147::CalibrationTools::Clear_and_prepare_stage_settings(uint8_t stage) {
    StageConfig config = pthis->stage_settings_.stages[stage];
    config.afe_offset.bits.pos_afe_offset_swap = 0;
    config.afe_offset.bits.pos_afe_offset = 0;
    config.afe_offset.bits.neg_afe_offset_swap = 0;
    config.afe_offset.bits.neg_afe_offset = 0;
    config.sensitivity.raw = AD7147_SENSITIVITY_DEFAULT;
    config.offset_high = 0;
    config.offset_low = 0;
    config.offset_high_clamp = 0xFFFF;
    config.offset_low_clamp = 0;
    pthis->setStageConfig(stage, config);
}

// 设置AFE偏移 0-127
void AD7147::CalibrationTools::Set_AEF_Offset(uint8_t stage, Direction direction, uint8_t offset) {
    StageConfig config = pthis->stage_settings_.stages[stage];
    if (direction == Pos) {
        config.afe_offset.bits.pos_afe_offset_swap = 0;
        config.afe_offset.bits.pos_afe_offset = offset & 0x3F;
        config.afe_offset.bits.neg_afe_offset_swap = 0;
        config.afe_offset.bits.neg_afe_offset = 0;
        if (offset > 63) {
            config.afe_offset.bits.neg_afe_offset_swap = 1;
            config.afe_offset.bits.neg_afe_offset = (offset - 64) & 0x3F;
        }
    } else {
        config.afe_offset.bits.neg_afe_offset_swap = 0;
        config.afe_offset.bits.neg_afe_offset = offset & 0x3F;
        config.afe_offset.bits.pos_afe_offset_swap = 0;
        config.afe_offset.bits.pos_afe_offset = 0;
        if (offset > 63) {
            config.afe_offset.bits.pos_afe_offset_swap = 1;
            config.afe_offset.bits.pos_afe_offset = (offset - 64) & 0x3F;
        }
    }
    pthis->setStageConfig(stage, config);
}

void AD7147::CalibrationTools::Set_Offset(uint8_t stage, Direction direction, uint16_t offset) {
    StageConfig config = pthis->stage_settings_.stages[stage];
    if (direction == Pos) {
        config.offset_high = offset;
    } else {
        config.offset_low = offset;
    }
    pthis->setStageConfig(stage, config);
}

void AD7147::CalibrationTools::Set_Clamp(uint8_t stage, Direction direction, uint16_t clamp) {
    StageConfig config = pthis->stage_settings_.stages[stage];
    if (direction == Pos) {
        config.offset_high_clamp = clamp;
    } else {
        config.offset_low_clamp = clamp;
    }
    pthis->setStageConfig(stage, config);
}


bool AD7147::CalibrationTools::Read_CDC_50Sample(uint8_t stage, CDCSample_result& result) {
    static uint16_t value;
    pthis->readStageCDC(stage, value);
    if (result.sample_count) {
        result.average = (result.average * result.sample_count + value) / (result.sample_count + 1);
    } else result.average = value;
    result.max = MAX(result.max, value);
    result.min = MIN(result.min, value);
    return result.sample_count++ >= 50;
}

bool AD7147::CalibrationTools::Read_Triggle_50Sample(uint8_t stage, uint32_t sample, TriggleSample& result) {
    if (sample & ( 1 << stage )) result.triggle_num++; 
    else result.not_triggle_num++;
    return result.sample_count++ >= 50;
}