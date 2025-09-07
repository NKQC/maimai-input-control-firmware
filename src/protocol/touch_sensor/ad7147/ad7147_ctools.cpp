#include "ad7147.h"
#include <algorithm>

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
    pthis->register_config_.stage_cal_en.raw = 0x0FFF;
    uint8_t cal_en_data[2] = {
        (uint8_t)(pthis->register_config_.stage_cal_en.raw >> 8),
        (uint8_t)(pthis->register_config_.stage_cal_en.raw & 0xFF)
    };
    pthis->write_register(AD7147_REG_STAGE_CAL_EN, cal_en_data, 2);
}

void AD7147::CalibrationTools::Complete_and_restore_calibration() {
    pthis->register_config_.stage_cal_en.raw = 0x0000;
    uint8_t cal_en_data[2] = {
        (uint8_t)(pthis->register_config_.stage_cal_en.raw >> 8),
        (uint8_t)(pthis->register_config_.stage_cal_en.raw & 0xFF)
    };
    pthis->write_register(AD7147_REG_STAGE_CAL_EN, cal_en_data, 2);
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


bool AD7147::CalibrationTools::Read_CDC_Sample(uint8_t stage, CDCSample_result& result) {
    static uint16_t value;
    pthis->readStageCDC_direct(stage, value);
    if (result.sample_count) {
        result.average = (uint16_t)((result.average * result.sample_count + value) / (result.sample_count + 1));
    } else result.average = value;
    result.max = MAX(result.max, value);
    result.min = MIN(result.min, value);
    return result.sample_count++ >= 100;
}

bool AD7147::CalibrationTools::Read_Triggle_Sample(uint8_t stage, uint32_t sample, TriggleSample& result) {
    if (sample & ( 1u << stage )) result.triggle_num++; 
    else result.not_triggle_num++;
    return result.sample_count++ >= 100;
}

void AD7147::CalibrationTools::CalibrationLoop(uint32_t sample) {
    // 静态变量：全流程状态与本阶段临时采样累积（不在类中保存中间采样状态）
    static bool start = false;
    static uint8_t stage_index = 0;         // 当前校准的stage [0..11]

    static Direction aef_dir = Pos;         // AEF当前调整方向
    static int16_t aef_offset = 127;        // AEF偏移 0..127
    static Stage1Mode stage1_mode = FOUND_BASELINE;
    static int16_t balance_coefficient = 0; // 平衡系数 用于设置阶段1采样的稳定度趋势 0=触发与非触发处于接近50% 每-1 则触发比值-10% 每+1 触发比值+10%

    static uint16_t s1_min_abs = 0xFFFF;    // 最小绝对误差
    static int16_t s1_best_idx = 0;         // 最佳中线值 对应索引[-127..127]，>=0表示Pos，<0表示Neg
    static int16_t s1_triggle_best_idx = 0; // 最佳触发值 对应索引[-127..127]，>=0表示Pos，<0表示Neg

    static bool stage2_phase_raise = true;  // 阶段2：初始抬高阈值消除触发
    static uint16_t stage2_offset = 0;      // 阶段2：当前高偏移
    static uint16_t stage2_step = 50;       // 阶段2：步进
    static uint8_t stage2_halve_cnt = 0;    // 阶段2：折半次数（<=5）

    // 阶段2回退与首次进入标记
    static uint8_t stage2_retry_cnt = 0;    // 阶段2：回退计数（每个stage最多5次）
    static bool    stage2_first_entry = false; // 进入阶段2后的首次检测标记
    
    static uint16_t verify_cdc_min = 0xFFFF, verify_cdc_max = 0; // 阶段2验证期的CDC范围

    static CDCSample_result cdc_res;        // 公共CDC采样缓存
    static TriggleSample trig_res;          // 公共触发采样缓存

    const int32_t baseline = AD7147_CDC_BASELINE;

    // 在任何状态下，都同步当前stage索引（IDLE时可能为0或12）
    current_stage_index_ = stage_index;

    // CDC转换连续值
    static auto idx_to_dir_off = [&](int16_t sval){
        static uint8_t write_offset;
        if (sval >= 0) { aef_dir = Pos; write_offset = (uint8_t)((sval > 127) ? 127 : sval); }
        else { aef_dir = Neg; write_offset = (uint8_t)((-write_offset > 127) ? 127 : -write_offset); }
        Set_AEF_Offset(stage_index, aef_dir, write_offset);
    };

    // 初始化
    if (!start) {
        start = true;
        Clear_and_prepare_stage_settings(stage_index);
    }
    
    switch (calibration_state_) {
        case Stage1_baseline: {
            // 阶段1：

            auto reset_stage1 = [&](){
                s1_min_abs = 0xFFFF;
                aef_offset = 127;
                stage1_mode = FOUND_BASELINE;
            };

            // 模式切换
            switch (stage1_mode) {
                case FOUND_BASELINE: {
                    if (!Read_CDC_Sample(stage_index, cdc_res)) return;

                    int32_t err = abs((int32_t)cdc_res.average - baseline);
                    // 更新最优：仅当更小的绝对误差时更新
                    if (err < s1_min_abs) {
                        s1_min_abs = err;
                        s1_best_idx = aef_offset;
                    }
                    cdc_res.clear();

                    // 推进扫描
                    // 初次全扫：Pos 127->0，再 Neg 0->127
                    idx_to_dir_off((aef_offset--));
                    stage_process = 10;
                    if (aef_offset > (int16_t)-127) return; 
                    // 全扫结束：采用最终最优点
                    idx_to_dir_off(s1_best_idx);
                    stage1_mode = FOUND_TRIGGLELINE;
                    stage_process = 32;
                    break;
                }
                case FOUND_TRIGGLELINE: {
                    // 在基线s1_best_idx附近±49范围内进行震荡扫描，纯整数运算，无浮点
                    // 目标：使触发占比尽可能接近期望值（50% + balance_coefficient*10%）
                    static bool s1_scan_inited = false;
                    static uint8_t s1_scan_substate = 0;      // 0: 设置目标点并清采样; 1: 采样并评估
                    static int16_t s1_scan_target_idx = 0;     // 当前硬件下发的signed AEF索引
                    static int16_t s1_scan_base_idx = 0;       // 基线缓存
                    static uint32_t s1_best_metric = 0xFFFFFFFFu; // 最优度量（越小越好）：|trig*100 - cnt*target%|
                    // 两退一进序列状态：从-49开始，前一步进(+2)，后退一步(-1)，循环，直到+49
                    static int16_t s1_scan_delta = -49;        // 当前相对基线偏移
                    static uint8_t s1_step_forward = 1;        // 1: +2 toward +方向；0: -1 back
                    
                    // 初始化一次
                    if (!s1_scan_inited) {
                        s1_scan_inited = true;
                        s1_scan_substate = 0;
                        s1_best_metric = 0xFFFFFFFFu;
                        s1_scan_base_idx = s1_best_idx; // 基线缓存，扫描过程中保持不变
                        s1_triggle_best_idx = s1_best_idx; // 先用基线作为候选
                        s1_scan_delta = -49;
                        s1_step_forward = 1; // 先+2，再-1
                        trig_res.clear();
                    }

                    // 子状态0：设置目标点并清空采样累积
                    if (s1_scan_substate == 0) {
                        int16_t idx = (int16_t)(s1_scan_base_idx + s1_scan_delta);
                        if (idx > 127) idx = 127; else if (idx < -127) idx = -127;
                        s1_scan_target_idx = idx;
                        // 下发硬件设置，清空触发采样累积
                        idx_to_dir_off(s1_scan_target_idx);
                        trig_res.clear();
                        s1_scan_substate = 1;
                        stage_process = 48;
                        return;
                    }

                    // 子状态1：采样并评估该点
                    if (!Read_Triggle_Sample(stage_index, sample, trig_res)) return; // 未达到统计周期，继续采样
                    {
                        // 以整数方式计算与期望触发占比的偏差：最小化 |trig*100 - cnt*target_percent|
                        uint16_t cnt = trig_res.sample_count;
                        uint16_t trig = trig_res.triggle_num;
                        int16_t target_percent = (int16_t)(50 + (int16_t)balance_coefficient * 10); // 期望百分比[0..100]
                        if (target_percent < 0) target_percent = 0; else if (target_percent > 100) target_percent = 100;
                        uint32_t left = (uint32_t)trig * 100u;
                        uint32_t right = (uint32_t)cnt * (uint32_t)target_percent;
                        uint32_t metric = (left >= right) ? (left - right) : (right - left);

                        // 采用“更小metric优先；若相等，选择离基线绝对距离更小；再相等则选择较小索引”
                        int16_t base = s1_scan_base_idx;
                        int16_t cur = s1_scan_target_idx;
                        uint16_t cur_dist = (uint16_t)( (cur >= base) ? (cur - base) : (base - cur) );
                        uint16_t best_dist = (uint16_t)( (s1_triggle_best_idx >= base) ? (s1_triggle_best_idx - base) : (base - s1_triggle_best_idx) );

                        bool better = false;
                        if (metric < s1_best_metric) better = true;
                        else if (metric == s1_best_metric) {
                            if (cur_dist < best_dist) better = true;
                            else if (cur_dist == best_dist && cur < s1_triggle_best_idx) better = true;
                        }
                        if (better) {
                            s1_best_metric = metric;
                            s1_triggle_best_idx = cur;
                        }
                    }

                    // 若已在+49，扫描完成
                    if (s1_scan_delta == 49) {
                        s1_best_idx = s1_triggle_best_idx;
                        idx_to_dir_off(s1_best_idx);
                        trig_res.clear();
                        // 复位本地扫描状态
                        s1_scan_inited = false;
                        s1_scan_substate = 0;
                        s1_best_metric = 0xFFFFFFFFu;
                        // 为下一轮准备缺省序列状态
                        s1_scan_delta = -29;
                        s1_step_forward = 1;
                        stage1_mode = STAGE1_COMPLETE;
                        stage_process = 87;
                        break;
                    }

                    // 生成下一步：先+2，再-1，循环
                    if (s1_step_forward) {
                        s1_scan_delta = (int16_t)(s1_scan_delta + 2);
                        s1_step_forward = 0;
                    } else {
                        s1_scan_delta = (int16_t)(s1_scan_delta - 1);
                        s1_step_forward = 1;
                    }
                    s1_scan_substate = 0;
                    stage_process = 64;
                    break;
                }
                case STAGE1_COMPLETE: {
                    // 进入阶段2（偏移阈值校准）
                    trig_res.clear();
                    stage2_offset = pthis->stage_settings_.stages[stage_index].offset_high;
                    stage2_phase_raise = true;
                    stage2_step = 50;
                    stage2_halve_cnt = 0;
                    stage2_first_entry = true; // 标记首次进入阶段2
                    calibration_state_ = Stage2_Offset_calibration;
                    
                    // 复位Stage1状态
                    reset_stage1();

                    stage_process = 100;
                    return;
                }
            }
        }

        break;

        case Stage2_Offset_calibration: {

            // 阶段2（新规范）：
            // - 移除AEF偏移回退；
            // - 当正极极限且仍触发时：balance_coefficient -= 1，回退到阶段1；
            // - 当开局就未触发时：balance_coefficient += 1，回退到阶段1；
            // - 回退最多5次，超过则跳过该stage，进入下一个stage。

            // 回退到阶段1或跳过本stage的通用流程
            auto do_fallback_to_stage1 = [&](int8_t delta){
                balance_coefficient = (int16_t)(balance_coefficient + (int16_t)delta);
                stage2_retry_cnt++;

                // 清理采样缓存，复位阶段1本地状态
                trig_res.clear();
                cdc_res.clear();
                s1_min_abs = 0xFFFF; aef_offset = 127; stage1_mode = FOUND_BASELINE;

                if (stage2_retry_cnt <= 5) {
                    // 回到阶段1，同一个stage重试
                    calibration_state_ = Stage1_baseline;
                    // Stage2的首次进入标记将在下次进入阶段2时重新置位
                } else {
                    // 超过回退次数：跳过本stage，进入下一个
                    stage2_retry_cnt = 0; // 为后续stage重置计数
                    stage_index++;
                    if (stage_index >= 12) {
                        // 全部完成
                        Complete_and_restore_calibration();
                        current_stage_index_ = 12; // 用于“总进度”达到100%
                        calibration_state_ = IDLE;
                        return;
                    } else {
                        // 准备下一个stage
                        current_stage_index_ = stage_index; // 同步到新的stage
                        s1_best_idx = 0;
                        stage2_phase_raise = true;
                        stage2_offset = 0;
                        stage2_step = 50;
                        stage2_halve_cnt = 0;
                        stage2_retry_cnt = 0; // 重置回退计数
                        verify_cdc_min = 0xFFFF;
                        verify_cdc_max = 0;
                        
                        trig_res.clear();
                        Clear_and_prepare_stage_settings(stage_index);
                        calibration_state_ = Stage1_baseline;
                        return;
                    }
                }
            };

            // 保留原有的触发采样流程，但按照新规范处理回退与跳过
            if (stage2_phase_raise) {
                // 抬高阶段使用触发采样
                if (!Read_Triggle_Sample(stage_index, sample, trig_res)) return;
                if (trig_res.triggle_num != 0) {
                    // 已触发：继续抬高高偏移使其不触发；若到正极极限仍触发则回退到阶段1且balance_coefficient-1
                    if (stage2_offset < 0xFFFFu) {
                        uint32_t nxt = (uint32_t)stage2_offset + (uint32_t)stage2_step; // 默认步长50
                        if (nxt > 0xFFFFu) nxt = 0xFFFFu;
                        stage2_offset = (uint16_t)nxt;
                        Set_Offset(stage_index, Pos, stage2_offset);
                        stage_process = 130;
                    } else {
                        // 到正极极限值且仍触发：balance_coefficient -1，然后回退到阶段1（或跳过本stage）
                        do_fallback_to_stage1(-1);
                        trig_res.clear();
                        return;
                    }
                } else {
                    // 未出现触发
                    if (stage2_first_entry) {
                        // 开局就没触发：balance_coefficient +1，然后回退到阶段1（或跳过本stage）
                        stage2_first_entry = false;
                        do_fallback_to_stage1(+1);
                        trig_res.clear();
                        return;
                    }
                    // 非首帧：按照原逻辑进入细扫与验证
                    stage2_phase_raise = false;
                    stage2_halve_cnt = 3; // 子阶段3：向下细扫
                    stage_process = 180;
                    if (stage2_offset) {
                        stage2_offset = (uint16_t)(stage2_offset - 1);
                        Set_Offset(stage_index, Pos, stage2_offset);
                    } else {
                        // 低偏移到极限：直接进入验证阶段（不使用AEF回退）
                        calibration_state_ = Stage3_verify_limit;
                        verify_cdc_min = 0xFFFF; verify_cdc_max = 0;
                    }
                }
                trig_res.clear();
                return;
            } else {
                if (stage2_halve_cnt == 2) {
                    // AEF回退模式已移除，不再进入此分支；直接进入验证阶段
                    calibration_state_ = Stage3_verify_limit;
                    verify_cdc_min = 0xFFFF; verify_cdc_max = 0;
                    trig_res.clear();
                    return;
                } else if (stage2_halve_cnt == 3) {
                    // 子阶段3：向下细扫（步长1），直到不触发 -> 进入验证阶段
                    if (!Read_Triggle_Sample(stage_index, sample, trig_res)) return;
                    if (trig_res.triggle_num > 0) {
                        if (stage2_offset > 0) {
                            stage2_offset = (uint16_t)(stage2_offset - 1);
                            Set_Offset(stage_index, Pos, stage2_offset);
                        } else {
                            // 低偏移到极限：直接进入验证阶段（不使用AEF回退）
                            calibration_state_ = Stage3_verify_limit;
                            verify_cdc_min = 0xFFFF; verify_cdc_max = 0;
                        }
                    } else {
                        // 找到第一个不触发点，进入验证阶段（仅验证不触发）
                        calibration_state_ = Stage3_verify_limit;
                        verify_cdc_min = 0xFFFF; verify_cdc_max = 0; // CDC不再统计
                    }
                    trig_res.clear();
                    stage_process = 200;
                    return;
                } else {
                    // 子阶段1：向上细扫（步长1），直到不触发 -> 进入下一阶段
                    if (!Read_Triggle_Sample(stage_index, sample, trig_res)) return;
                    if (trig_res.triggle_num > 0) {
                        if (stage2_offset < 0xFFFFu) {
                            stage2_offset = (uint16_t)(stage2_offset + 1);
                            Set_Offset(stage_index, Pos, stage2_offset);
                        } else {
                            // 高偏移到极限仍触发：balance_coefficient -1，然后回退到阶段1（或跳过本stage）
                            do_fallback_to_stage1(-1);
                            trig_res.clear();
                            return;
                        }
                    } else {
                        // 找到第一个不触发点，进入验证阶段（仅验证不触发）
                        calibration_state_ = Stage3_verify_limit;
                        verify_cdc_min = 0xFFFF; verify_cdc_max = 0; // CDC不再统计
                    }
                    trig_res.clear();
                    return;
                }
            }
        }

        break;

        case Stage3_verify_limit: {
            // 根据阶段2验证的CDC范围设置Clamp
            uint32_t low_clamp = (uint32_t)((verify_cdc_min * 80u) / 100u);
            uint32_t high_clamp = (uint32_t)verify_cdc_max * 2u;
            if (high_clamp > 0xFFFFu) high_clamp = 0xFFFFu;
            Set_Clamp(stage_index, Neg, (uint16_t)std::min<uint32_t>(0xFFFFu, low_clamp));
            Set_Clamp(stage_index, Pos, (uint16_t)std::min<uint32_t>(0xFFFFu, high_clamp));
            stage_process = 250;
            // 切换到下一个stage或结束
            stage_index++;
            if (stage_index >= 12) {
                // 全部完成
                Complete_and_restore_calibration();
                current_stage_index_ = 12; // 用于“总进度”达到100%
                calibration_state_ = IDLE;
                return;
            } else {
                // 准备下一个stage
                current_stage_index_ = stage_index; // 同步到新的stage
                s1_best_idx = 0;
                stage2_phase_raise = true;
                stage2_offset = 0;
                stage2_step = 50;
                stage2_halve_cnt = 0;
                verify_cdc_min = 0xFFFF;
                verify_cdc_max = 0;
                
                trig_res.clear();
                Clear_and_prepare_stage_settings(stage_index);
                calibration_state_ = Stage1_baseline;
                return;
            }
        }

        default:
            return;
    }
}