/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

// #include <string.h>
// #include "nrf_drv_pwm.h"
// #include "drv_ir.h"
// #include "autil_platform.h"
// #include "adebug.h"

#include <nrfx_pwm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv_ir, LOG_LEVEL_DBG);


// #define NRF_LOG_MODULE_NAME drv_ir_nec
// #define NRF_LOG_LEVEL CONFIG_IR_DRV_LOG_LEVEL
// #include "nrf_log.h"
// NRF_LOG_MODULE_REGISTER();

// #define COMMAND_BITS        8
// #define NEC_ADDRESS_BITS        8
#define NEC_GUARD_ZEROS         1
#define TRAILING_SEQUENCE_BITS  8
#define FRAME_BITS              28
#define EXT_FRAME_BITS          32

#define NEC_TOP_VALUE           421      // 38kHz carrier 26.3125 usec for 16MHz PWM_CLK
#define NEC_SYMBOL_REPEATS      21       // repeat each symbol 21 times - 21*26.3125=552.5625 us

#define NEC_MARK_SYMBOL         0x808C   // 140/421 (1/16000000 sec)  - duty 1/3
#define NEC_SPACE_SYMBOL        0x8000

#define NEC_REPETION_MAX        91       // X times with 110 ms period
#define NEC_REPETION_TIME       1760000  // 1760000 / 16 MHz = 110 ms
#define NEC_REPETION_PERIODS    (NEC_REPETION_TIME / NEC_TOP_VALUE)

#define PWM_INST_IDX            0

// 9 ms high, 4.4 ms low
const uint16_t NEC_START[] =
    { NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL};

const uint16_t TRAILING_SEQUENCE[] =
    { NEC_MARK_SYMBOL,  NEC_SPACE_SYMBOL,  NEC_MARK_SYMBOL,  NEC_SPACE_SYMBOL, \
      NEC_SPACE_SYMBOL,  NEC_MARK_SYMBOL,  NEC_SPACE_SYMBOL,  NEC_MARK_SYMBOL};

// 19 ms low
const uint16_t EXT_FRAME_SPACE[] =
    { NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL};

const uint16_t NEC_ONE[] =
    { NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL , NEC_SPACE_SYMBOL };

const uint16_t NEC_ZERO[] =
    { NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL };

#define MAX_SEQ_SIZE            ARRAY_SIZE(NEC_START) + \
                                (FRAME_BITS) * MAX(ARRAY_SIZE(NEC_ZERO), ARRAY_SIZE(NEC_ONE)) + \
                                ARRAY_SIZE(TRAILING_SEQUENCE) + \
                                ARRAY_SIZE(EXT_FRAME_SPACE) + \
                                (EXT_FRAME_BITS) * MAX(ARRAY_SIZE(NEC_ZERO), ARRAY_SIZE(NEC_ONE)) + \
                                (NEC_GUARD_ZEROS * ARRAY_SIZE(NEC_ZERO))


union frame
{
    uint32_t content;
    struct {
        uint32_t mode : 3;
        uint32_t on : 1;
        uint32_t fan : 2;
        uint32_t oscillating : 1;
        uint32_t sleep : 1;
        uint32_t temperature : 4;
        uint32_t timer : 8;
        uint32_t turbo : 1;
        uint32_t light : 1;
        uint32_t pine_tree : 1;
        uint32_t x_fan : 1;
        uint32_t scavenging : 1;
        uint32_t : 2;
        uint32_t set_config : 1;
    }
};

union ext_frame
{
    uint32_t content;
    struct {
        uint32_t swing : 4;
        uint32_t : 4;
        uint32_t temp : 4;
        uint32_t i_feel : 1;
        uint32_t : 1;
        uint32_t unknown : 4;
        uint32_t : 12;
        uint32_t temperature : 4;
    }
};


// static nrf_drv_pwm_t            pwm = CONFIG_IR_TX_PWM_INSTANCE;
static nrfx_pwm_t               pwm = NRFX_PWM_INSTANCE(PWM_INST_IDX);
static uint16_t                 seq_pwm_values[MAX_SEQ_SIZE];
static nrf_pwm_sequence_t       seq;
static drv_ir_callback_t        acknowledge_handler;
static const union frame        *frame;
static const union ext_frame    *ext_frame;
static bool                     pwm_active;
static bool                     enabled_flag;  

static void insert_start_symbol(uint16_t **seq)
{
    memcpy(*seq, NEC_START, sizeof(NEC_START));
    *seq += ARRAY_SIZE(NEC_START);
}

static void insert_trailing_sequence(uint16_t **seq)
{
    memcpy(*seq, TRAILING_SEQUENCE, sizeof(TRAILING_SEQUENCE));
    *seq += ARRAY_SIZE(TRAILING_SEQUENCE);
}

static void insert_ext_frame_space(uint16_t **seq)
{
    memcpy(*seq, EXT_FRAME_SPACE, sizeof(EXT_FRAME_SPACE));
    *seq += ARRAY_SIZE(EXT_FRAME_SPACE);
}

static void insert_one_symbol(uint16_t **seq)
{
    memcpy(*seq, NEC_ONE, sizeof(NEC_ONE));
    *seq += ARRAY_SIZE(NEC_ONE);
}

static void insert_zero_symbol(uint16_t **seq)
{
    memcpy(*seq, NEC_ZERO, sizeof(NEC_ZERO));
    *seq += ARRAY_SIZE(NEC_ZERO);
}

static void frame_encoder_process(uint32_t val, size_t len, uint16_t **seq)
{
    uint32_t local;

    local = val;

    for (int i = 0; i < len; i++)
    {
        if (local & 0x01)
        {
            insert_one_symbol(seq);
        }
        else
        {
            insert_zero_symbol(seq);
        }

        local = local >> 1;
    }
}
static uint16_t frame_encoder(const frame *p_frame, const ext_frame *p_ext_frame)
{
    uint16_t *seq;
    int i;

    seq = seq_pwm_values;
    insert_start_symbol(&seq);

    frame_encoder_process_byte(frame->content, FRAME_BITS, &seq);
    insert_trailing_sequence(&seq);

    insert_ext_frame_space(&seq);

    frame_encoder_process_byte(ext_frame->content, EXT_FRAME_BITS, &seq);

    for (i = 0; i < NEC_GUARD_ZEROS; i++)
    {
        insert_zero_symbol(&seq);
    }

    return (uint16_t)(seq - seq_pwm_values);
}

// static uint16_t nec_repeat_symbol_encoder(void)
// {
//     uint16_t *seq;

//     seq = &seq_pwm_values[0];

//     memcpy(seq, NEC_REPEAT, sizeof(NEC_REPEAT));
//     seq += ARRAY_SIZE(NEC_REPEAT);

//     // Set remaining values in PWM table to space
//     for (int i = 0; i < (ARRAY_SIZE(seq_pwm_values) - ARRAY_SIZE(NEC_REPEAT)); ++i)
//     {
//         seq[i] = NEC_SPACE_SYMBOL;
//     }

//     return (uint16_t)(seq - seq_pwm_values);
// }

static void pwm_handler(nrf_drv_pwm_evt_type_t event)
{
    DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_PWM_INT);

    if (((event == NRF_DRV_PWM_EVT_END_SEQ0) || (event == NRF_DRV_PWM_EVT_END_SEQ1)) && (ir_symbol == NULL))
    {
        nrf_drv_pwm_stop(&pwm, true); // Stop during repetition gap.
        acknowledge_handler(NULL);    // Acknowledge end.
        pwm_active = false;

        DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
    }
    else if ((event == NRF_DRV_PWM_EVT_END_SEQ0) || (event == NRF_DRV_PWM_EVT_END_SEQ1))
    {
        uint16_t seq_len;

        seq_len = nec_repeat_symbol_encoder();
        APP_ERROR_CHECK_BOOL(seq_len > 0 && seq_len < MAX_SEQ_SIZE);
    }

    if (event == NRF_DRV_PWM_EVT_FINISHED)
    {
        if (ir_symbol == NULL)
        {
            // Acknowledge end.
            acknowledge_handler(NULL);

            DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
        }
        else
        {
            __NOP();
        }

        pwm_active = false;
    }
}

ret_code_t drv_ir_send_symbol(const sr3_ir_symbol_t *p_ir_symbol)
{
    bool callback_flag = false;
    uint16_t seq_length;

    if (enabled_flag != true)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    CRITICAL_REGION_ENTER();
    ir_symbol = p_ir_symbol;
    if ((ir_symbol == NULL) && !pwm_active)
    {
        callback_flag = true;
    }
    CRITICAL_REGION_EXIT();

    if (callback_flag)
    {
        // Acknowledge of prematurely ended sequence - it won't be acknowledge by handler.
        acknowledge_handler(NULL);

        DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
    }
    else if (ir_symbol)
    {
        seq_length = nec_symbol_encoder(ir_symbol);

        if (seq_length > 0)
        {
            seq.values.p_common   = seq_pwm_values;
            seq.length            = seq_length;
            seq.repeats           = NEC_SYMBOL_REPEATS;
            seq.end_delay         = NEC_REPETION_PERIODS - ((NEC_SYMBOL_REPEATS + 1) * seq_length);
        }
        else
        {
            return NRF_ERROR_NOT_SUPPORTED;
        }

        pwm_active = true;
        nrf_drv_pwm_simple_playback(&pwm, &seq, NEC_REPETION_MAX,
                                    NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);

        acknowledge_handler(ir_symbol);
        DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_SACK);
    }

    return NRF_SUCCESS;
}

ret_code_t drv_ir_enable(void)
{
    ASSERT(enabled_flag == false);

    enabled_flag = true;
    nrf_pwm_enable(pwm.p_registers);

    return NRF_SUCCESS;
}

ret_code_t drv_ir_disable(void)
{
    ASSERT(enabled_flag == true);

    nrf_pwm_disable(pwm.p_registers);
    enabled_flag = false;

    return NRF_SUCCESS;
}

ret_code_t drv_ir_init(drv_ir_callback_t acknowledge_handler)
{
    ret_code_t status;

    static const nrf_drv_pwm_config_t config =
    {
        .output_pins =
        {
            IS_IO_VALID(CONFIG_IO_IR_TX_LED) ? CONFIG_IO_IR_TX_LED : NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
        },

        .irq_priority   = AIRQ_PRIORITY_LOW,
        .base_clock     = NRF_PWM_CLK_16MHz,
        .count_mode     = NRF_PWM_MODE_UP,
        .top_value      = NEC_TOP_VALUE,
        .load_mode      = NRF_PWM_LOAD_COMMON,
        .step_mode      = NRF_PWM_STEP_AUTO
    };

    if (acknowledge_handler == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    acknowledge_handler   = acknowledge_handler;
    enabled_flag          = false;
    pwm_active            = false;
    ir_symbol             = NULL;

    status = nrf_drv_pwm_init(&pwm, &config, pwm_handler);
    if (status == NRF_SUCCESS)
    {
        nrf_pwm_disable(pwm.p_registers);
    }

    return status;
}
