#include "fast_trigger.h"

typedef struct {
    uint16_t baseline;
    uint16_t x_delta;
    int32_t  z_h;
    int32_t  z_l;
    uint64_t z_ht;
    uint64_t z_lt;
    bool     need_reset;
} fast_ch_t;

static fast_ch_t g_ch[CAPSENSE_WIDGET_COUNT];
static uint16_t g_x_permille = FAST_TRIG_X_PERMILLE_DEFAULT;
static uint16_t _enable_mask = 0xFFFFu;
static uint16_t _override_release_mask = 0u;

static int32_t d = 0;
static uint64_t _s_now;
static uint16_t _s_out;
static uint8_t  _s_i;
static fast_ch_t* _s_ch;


void fast_trigger_init(void)
{
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        fast_ch_t* ch = &g_ch[i];
        ch->baseline = capsense_get_baseline(i);
        ch->x_delta  = (uint16_t)((ch->baseline * g_x_permille) / 1000u);
        ch->need_reset = true;
    }
    _override_release_mask = 0xFFFFu;
}

uint16_t fast_trigger_process(uint64_t now_ms, uint16_t base_status)
{
    _s_now = now_ms;
    _s_out = base_status;
    _override_release_mask = 0xFFFFu;
    for (_s_i = 0; _s_i < CAPSENSE_WIDGET_COUNT; ++_s_i) {
        _s_ch = &g_ch[_s_i];
        
        {
            uint16_t b = capsense_get_baseline(_s_i);
            if (b != _s_ch->baseline) {
                _s_ch->baseline = b;
                _s_ch->x_delta  = (uint16_t)((_s_ch->baseline * g_x_permille) / 1000u);
            }
        }

        if ((base_status >> _s_i) & 0x1u) {
            if (_s_ch->need_reset) {
                _s_ch->need_reset = false;
                _s_ch->z_h = FAST_TRIG_INVALID_HIGH; _s_ch->z_l = FAST_TRIG_INVALID_LOW;
                _s_ch->z_ht = _s_now; _s_ch->z_lt = _s_now;
                _override_release_mask |= (uint16_t)(1u << _s_i);
            }
            d = (int32_t)capsense_get_raw_filtered(_s_i) - (int32_t)_s_ch->baseline;
            
#ifndef FAST_TRIG_BASE_NEXT_STATE
            if ((_s_now - _s_ch->z_ht) > FAST_TRIG_WINDOW_MS) { 
                _s_ch->z_h = FAST_TRIG_INVALID_HIGH; _s_ch->z_ht = 0;
            }
            if ((_s_now - _s_ch->z_lt) > FAST_TRIG_WINDOW_MS) { 
                _s_ch->z_l = FAST_TRIG_INVALID_LOW; _s_ch->z_lt = 0; 
            }
#endif
            if (d > _s_ch->z_h) { 
                _s_ch->z_h = d; _s_ch->z_ht = _s_now; 
            }
            if (d < _s_ch->z_l) { 
                _s_ch->z_l = d; _s_ch->z_lt = _s_now; 
            }
            {
                uint64_t latest_ts = 0;
                int32_t  latest_val = 0;
                uint64_t oldest_ts = 0;
                int32_t  oldest_val = 0;

                if (_s_ch->z_ht != 0 && (_s_ch->z_ht >= _s_ch->z_lt)) { latest_ts = _s_ch->z_ht; latest_val = _s_ch->z_h; }
                else if (_s_ch->z_lt != 0) { latest_ts = _s_ch->z_lt; latest_val = _s_ch->z_l; }

                if (_s_ch->z_ht != 0 && (_s_ch->z_lt == 0 || _s_ch->z_ht <= _s_ch->z_lt)) { oldest_ts = _s_ch->z_ht; oldest_val = _s_ch->z_h; }
                else if (_s_ch->z_lt != 0) { oldest_ts = _s_ch->z_lt; oldest_val = _s_ch->z_l; }

                if (latest_ts != 0 && oldest_ts != 0) {
                    int32_t diff = oldest_val - latest_val;
                    if ((diff >= 0 ? diff : -diff) > (uint32_t)_s_ch->x_delta) {
#ifdef FAST_TRIG_BASE_NEXT_STATE
                        if (diff > 0) {
                            _override_release_mask &= (uint16_t)~(1u << _s_i);
                            _s_ch->z_h = FAST_TRIG_INVALID_HIGH; _s_ch->z_ht = 0;
                        } else {
                            _override_release_mask |= (uint16_t)(1u << _s_i);
                            _s_ch->z_l = FAST_TRIG_INVALID_LOW; _s_ch->z_lt = 0;
                        }
#else
                        if (diff > 0) {
                            _override_release_mask &= (uint16_t)~(1u << _s_i);
                        } else {
                            _override_release_mask |= (uint16_t)(1u << _s_i);
                        }
#endif
                    }
                }
            }
        } else {
            _s_ch->need_reset = true;
        }
    }
    return (uint16_t)(_s_out & _override_release_mask);
}



void fast_trigger_set_x_permille(uint16_t p)
{
    g_x_permille = p;
    for (uint8_t i = 0; i < CAPSENSE_WIDGET_COUNT; ++i) {
        g_ch[i].x_delta = (uint16_t)((g_ch[i].baseline * g_x_permille) / 1000u);
    }
}

void fast_trigger_set_enable_mask(uint16_t mask)
{
    _enable_mask = mask;
}

uint16_t fast_trigger_get_x_permille(void) { return g_x_permille; }
uint16_t fast_trigger_get_enable_mask(void) { return _enable_mask; }