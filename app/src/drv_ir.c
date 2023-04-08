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

#include <string.h>
#include "nrf_drv_pwm.h"
#include "drv_ir.h"
#include "app_util_platform.h"
#include "app_debug.h"

#if CONFIG_IR_TX_ENABLED && (CONFIG_IR_PROTOCOL == CONFIG_IR_TX_PROTOCOL_NEC)

#define NRF_LOG_MODULE_NAME drv_ir_nec
#define NRF_LOG_LEVEL CONFIG_IR_DRV_LOG_LEVEL
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

// Verify SDK driver configuration.
STATIC_ASSERT(PWM_ENABLED && PWM0_ENABLED);

#define NEC_COMMAND_BITS        8
#define NEC_ADDRESS_BITS        8
#define NEC_GUARD_ZEROS         1

#define NEC_TOP_VALUE           421      // 38kHz carrier 26.3125 usec for 16MHz PWM_CLK
#define NEC_SYMBOL_REPEATS      21       // repeat each symbol 21 times - 21*26.3125=552.5625 us

#define NEC_MARK_SYMBOL         0x808C   // 140/421 (1/16000000 sec)  - duty 1/3
#define NEC_SPACE_SYMBOL        0x8000

#define NEC_REPETION_MAX        91       // X times with 110 ms period
#define NEC_REPETION_TIME       1760000  // 1760000 / 16 MHz = 110 ms
#define NEC_REPETION_PERIODS    (NEC_REPETION_TIME / NEC_TOP_VALUE)

const uint16_t NEC_START[] =
    { NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL};

const uint16_t NEC_REPEAT[] =
    { NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
      NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
      NEC_MARK_SYMBOL};

STATIC_ASSERT(ARRAY_SIZE(NEC_START) >= ARRAY_SIZE(NEC_REPEAT));

const uint16_t NEC_ONE[] =
    { NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL , NEC_SPACE_SYMBOL };

const uint16_t NEC_ZERO[] =
    { NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL };

#define MAX_SEQ_SIZE            ARRAY_SIZE(NEC_START) + \
                                ((NEC_COMMAND_BITS + NEC_ADDRESS_BITS) * MAX(ARRAY_SIZE(NEC_ZERO), ARRAY_SIZE(NEC_ONE)) * 2) + \
                                (NEC_GUARD_ZEROS * ARRAY_SIZE(NEC_ZERO))

static nrf_drv_pwm_t            m_pwm = CONFIG_IR_TX_PWM_INSTANCE;
static uint16_t                 m_seq_pwm_values[MAX_SEQ_SIZE];
static nrf_pwm_sequence_t       m_seq;
static drv_ir_callback_t        m_acknowledge_handler;
static const sr3_ir_symbol_t    *mp_ir_symbol;
static bool                     m_pwm_active;
static bool                     m_enabled_flag;

static void insert_start_symbol(uint16_t **pp_seq)
{
    memcpy(*pp_seq, NEC_START, sizeof(NEC_START));
    *pp_seq += ARRAY_SIZE(NEC_START);
}

static void insert_one_symbol(uint16_t **pp_seq)
{
    memcpy(*pp_seq, NEC_ONE, sizeof(NEC_ONE));
    *pp_seq += ARRAY_SIZE(NEC_ONE);
}

static void insert_zero_symbol(uint16_t **pp_seq)
{
    memcpy(*pp_seq, NEC_ZERO, sizeof(NEC_ZERO));
    *pp_seq += ARRAY_SIZE(NEC_ZERO);
}

static void nec_symbol_encoder_process_byte(uint8_t val, uint16_t **pp_seq)
{
    uint8_t byte;

    byte = val;

    for (int i = 0; i < NEC_COMMAND_BITS; i++)
    {
        if (byte & 0x01)
        {
          insert_one_symbol(pp_seq);
        }
        else
        {
          insert_zero_symbol(pp_seq);
        }

        byte = byte >> 1;
    }
}

static uint16_t nec_symbol_encoder(const sr3_ir_symbol_t *p_nec_symbol)
{
    uint16_t *p_seq;
    int i;

    p_seq = m_seq_pwm_values;
    insert_start_symbol(&p_seq);

    nec_symbol_encoder_process_byte(p_nec_symbol->ir_address, &p_seq);
    nec_symbol_encoder_process_byte(p_nec_symbol->ir_address ^ 0xFF, &p_seq);
    nec_symbol_encoder_process_byte(p_nec_symbol->ir_command, &p_seq);
    nec_symbol_encoder_process_byte(p_nec_symbol->ir_command ^ 0xFF, &p_seq);

    for (i = 0; i < NEC_GUARD_ZEROS; i++)
    {
        // Trailing zero - to ensure 0 state when idle
        insert_zero_symbol(&p_seq);
    }

    return (uint16_t)(p_seq - m_seq_pwm_values);
}

static uint16_t nec_repeat_symbol_encoder(void)
{
    uint16_t *p_seq;

    p_seq = &m_seq_pwm_values[0];

    memcpy(p_seq, NEC_REPEAT, sizeof(NEC_REPEAT));
    p_seq += ARRAY_SIZE(NEC_REPEAT);

    // Set remaining values in PWM table to space
    for (int i = 0; i < (ARRAY_SIZE(m_seq_pwm_values) - ARRAY_SIZE(NEC_REPEAT)); ++i)
    {
        p_seq[i] = NEC_SPACE_SYMBOL;
    }

    return (uint16_t)(p_seq - m_seq_pwm_values);
}

static void pwm_handler(nrf_drv_pwm_evt_type_t event)
{
    DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_PWM_INT);

    if (((event == NRF_DRV_PWM_EVT_END_SEQ0) || (event == NRF_DRV_PWM_EVT_END_SEQ1)) && (mp_ir_symbol == NULL))
    {
        nrf_drv_pwm_stop(&m_pwm, true); // Stop during repetition gap.
        m_acknowledge_handler(NULL);    // Acknowledge end.
        m_pwm_active = false;

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
        if (mp_ir_symbol == NULL)
        {
            // Acknowledge end.
            m_acknowledge_handler(NULL);

            DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
        }
        else
        {
            __NOP();
        }

        m_pwm_active = false;
    }
}

ret_code_t drv_ir_send_symbol(const sr3_ir_symbol_t *p_ir_symbol)
{
    bool callback_flag = false;
    uint16_t seq_length;

    if (m_enabled_flag != true)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    CRITICAL_REGION_ENTER();
    mp_ir_symbol = p_ir_symbol;
    if ((mp_ir_symbol == NULL) && !m_pwm_active)
    {
        callback_flag = true;
    }
    CRITICAL_REGION_EXIT();

    if (callback_flag)
    {
        // Acknowledge of prematurely ended sequence - it won't be acknowledge by handler.
        m_acknowledge_handler(NULL);

        DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
    }
    else if (mp_ir_symbol)
    {
        seq_length = nec_symbol_encoder(mp_ir_symbol);

        if (seq_length > 0)
        {
            m_seq.values.p_common   = m_seq_pwm_values;
            m_seq.length            = seq_length;
            m_seq.repeats           = NEC_SYMBOL_REPEATS;
            m_seq.end_delay         = NEC_REPETION_PERIODS - ((NEC_SYMBOL_REPEATS + 1) * seq_length);
        }
        else
        {
            return NRF_ERROR_NOT_SUPPORTED;
        }

        m_pwm_active = true;
        nrf_drv_pwm_simple_playback(&m_pwm, &m_seq, NEC_REPETION_MAX,
                                    NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);

        m_acknowledge_handler(mp_ir_symbol);
        DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_SACK);
    }

    return NRF_SUCCESS;
}

ret_code_t drv_ir_enable(void)
{
    ASSERT(m_enabled_flag == false);

    m_enabled_flag = true;
    nrf_pwm_enable(m_pwm.p_registers);

    return NRF_SUCCESS;
}

ret_code_t drv_ir_disable(void)
{
    ASSERT(m_enabled_flag == true);

    nrf_pwm_disable(m_pwm.p_registers);
    m_enabled_flag = false;

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

        .irq_priority   = APP_IRQ_PRIORITY_LOW,
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

    m_acknowledge_handler   = acknowledge_handler;
    m_enabled_flag          = false;
    m_pwm_active            = false;
    mp_ir_symbol            = NULL;

    status = nrf_drv_pwm_init(&m_pwm, &config, pwm_handler);
    if (status == NRF_SUCCESS)
    {
        nrf_pwm_disable(m_pwm.p_registers);
    }

    return status;
}

#endif /* CONFIG_IR_TX_ENABLED && (CONFIG_IR_PROTOCOL == CONFIG_IR_TX_PROTOCOL_NEC) */
