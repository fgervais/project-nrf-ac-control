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
#include <zephyr/drivers/pinctrl.h>

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

// #define NEC_REPETION_MAX        91       // X times with 110 ms period
// #define NEC_REPETION_TIME       1760000  // 1760000 / 16 MHz = 110 ms
// #define  NEC_REPETION_TIME / NEC_TOP_VALUE)

// #define PWM_INST_IDX            0

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

#define MAX_SEQ_SIZE    ARRAY_SIZE(NEC_START) + \
			(FRAME_BITS) * MAX(ARRAY_SIZE(NEC_ZERO), ARRAY_SIZE(NEC_ONE)) + \
			ARRAY_SIZE(TRAILING_SEQUENCE) + \
			ARRAY_SIZE(EXT_FRAME_SPACE) + \
			(EXT_FRAME_BITS) * MAX(ARRAY_SIZE(NEC_ZERO), ARRAY_SIZE(NEC_ONE)) + \
			(NEC_GUARD_ZEROS * ARRAY_SIZE(NEC_ZERO))

#define DRV_IR_TEMP_BASE_VALUE		16

#define DRV_IR_FRAME_MODE_COOLING	0x1
#define DRV_IR_FRAME_FAN_AUTO		0x0
#define DRV_IR_FRAME_FAN_1		0x1
#define DRV_IR_FRAME_FAN_2		0x2
#define DRV_IR_FRAME_FAN_3		0x3
#define DRV_IR_FRAME_FAN_3		0x3

#define DRV_IR_EXT_FRAME_SWING_OFF	0x0
#define DRV_IR_EXT_FRAME_SWING_ALL	0x1
#define DRV_IR_EXT_FRAME_SWING_FIX_1	0x2 // Higher
#define DRV_IR_EXT_FRAME_SWING_FIX_2	0x3 // .
#define DRV_IR_EXT_FRAME_SWING_FIX_3	0x4 // .
#define DRV_IR_EXT_FRAME_SWING_FIX_4	0x5 // .
#define DRV_IR_EXT_FRAME_SWING_FIX_5	0x6 // Lower
#define DRV_IR_EXT_FRAME_SWING_LOW	0x7
#define DRV_IR_EXT_FRAME_SWING_MID	0x9
#define DRV_IR_EXT_FRAME_SWING_MID	0x9
#define DRV_IR_EXT_FRAME_TEMP_SHOW_ALL	0x0
#define DRV_IR_EXT_FRAME_TEMP_SET	0x1
#define DRV_IR_EXT_FRAME_TEMP_SHOW_IN	0x2
#define DRV_IR_EXT_FRAME_TEMP_SHOW_OUT	0x3
#define DRV_IR_EXT_FRAME_UNKNOWN_VAL	0x2
#define DRV_IR_EXT_FRAME_COOLING_TEMP_OFFSET_C	0x5


struct pwm_nrfx_config {
	nrfx_pwm_t pwm;
	nrfx_pwm_config_t initial_config;
	nrf_pwm_sequence_t seq;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

struct pwm_nrfx_data {
	uint32_t period_cycles;
	uint16_t seq_values[NRF_PWM_CHANNEL_COUNT];
	/* Bit mask indicating channels that need the PWM generation. */
	uint8_t  pwm_needed;
	uint8_t  prescaler;
	uint8_t  initially_inverted;
	bool     stop_requested;
	union drv_ir_frame	frame;
	union drv_ir_ext_frame	ext_frame;
};

union drv_ir_frame
{
	uint32_t content;
	struct {
		uint32_t mode		: 3;
		uint32_t on		: 1;
		uint32_t fan		: 2;
		uint32_t oscillating	: 1;
		uint32_t sleep		: 1;
		uint32_t temperature	: 4;
		uint32_t timer		: 8;
		uint32_t turbo		: 1;
		uint32_t light		: 1;
		uint32_t pine_tree	: 1;
		uint32_t x_fan		: 1;
		uint32_t scavenging	: 1;
		uint32_t		: 2;
		uint32_t set_config	: 1;
	}
};

union drv_ir_ext_frame
{
	uint32_t content;
	struct {
		uint32_t swing 		: 4;
		uint32_t 		: 4;
		uint32_t temp 		: 4;
		uint32_t i_feel 	: 1;
		uint32_t 		: 1;
		uint32_t unknown 	: 4;
		uint32_t 		: 12;
		uint32_t temperature 	: 4;
	}
};


// static nrf_drv_pwm_t            pwm = CONFIG_IR_TX_PWM_INSTANCE;
// static nrfx_pwm_t               pwm = NRFX_PWM_INSTANCE(PWM_INST_IDX);
static uint16_t                 seq_pwm_values[MAX_SEQ_SIZE];
// static nrf_pwm_sequence_t       seq;
// static drv_ir_callback_t        acknowledge_handler;
// static const union drv_ir_frame        *frame;
// static const union drv_ir_ext_frame    *ext_frame;
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

static uint16_t drv_ir_encode_frame(const union *frame, const union *ext_frame, const uint16_t *buf)
{
	uint16_t *seq;
	int i;

	const struct pwm_nrfx_config *config = dev->config;
	struct pwm_nrfx_data *data = dev->data;

	seq = config->seq;
	insert_start_symbol(&seq);

	frame_encoder_process_byte(data->frame.content, FRAME_BITS, &seq);
	insert_trailing_sequence(&seq);

	insert_ext_frame_space(&seq);

	frame_encoder_process_byte(data->ext_frame.content, EXT_FRAME_BITS, &seq);

	for (i = 0; i < NEC_GUARD_ZEROS; i++)
	{
		insert_zero_symbol(&seq);
	}

	return (uint16_t)(seq - config->seq);
}

static int drv_ir_transmit_sequence(const struct device *dev)
{
	/* We assume here that period_cycles will always be 16MHz
	 * peripheral clock. Since pwm_nrfx_get_cycles_per_sec() function might
	 * be removed, see ISSUE #6958.
	 * TODO: Remove this comment when issue has been resolved.
	 */
	const struct pwm_nrfx_config *config = dev->config;
	struct pwm_nrfx_data *data = dev->data;
	uint16_t compare_value;
	bool inverted = (flags & PWM_POLARITY_INVERTED);
	bool needs_pwm = false;

	if (channel >= NRF_PWM_CHANNEL_COUNT) {
		LOG_ERR("Invalid channel: %u.", channel);
		return -EINVAL;
	}

	/* If this PWM is in center-aligned mode, pulse and period lengths
	 * are effectively doubled by the up-down count, so halve them here
	 * to compensate.
	 */
	if (config->initial_config.count_mode == NRF_PWM_MODE_UP_AND_DOWN) {
		period_cycles /= 2;
		pulse_cycles /= 2;
	}

	if (pulse_cycles == 0) {
		/* Constantly inactive (duty 0%). */
		compare_value = 0;
	} else if (pulse_cycles >= period_cycles) {
		/* Constantly active (duty 100%). */
		/* This value is always greater than or equal to COUNTERTOP. */
		compare_value = PWM_NRFX_CH_COMPARE_MASK;
	} else {
		/* PWM generation needed. Check if the requested period matches
		 * the one that is currently set, or the PWM peripheral can be
		 * reconfigured accordingly.
		 */
		if (!pwm_period_check_and_set(dev, channel, period_cycles)) {
			return -EINVAL;
		}

		compare_value = (uint16_t)(pulse_cycles >> data->prescaler);
		needs_pwm = true;
	}

	data->seq_values[channel] = PWM_NRFX_CH_VALUE(compare_value, inverted);

	LOG_DBG("channel %u, pulse %u, period %u, prescaler: %u.",
		channel, pulse_cycles, period_cycles, data->prescaler);

	/* If this channel does not need to be driven by the PWM peripheral
	 * because its state is to be constant (duty 0% or 100%), set properly
	 * the GPIO configuration for its output pin. This will provide
	 * the correct output state for this channel when the PWM peripheral
	 * is stopped.
	 */
	if (!needs_pwm) {
		uint32_t psel;

		if (channel_psel_get(channel, &psel, config)) {
			uint32_t out_level = (pulse_cycles == 0) ? 0 : 1;

			if (inverted) {
				out_level ^= 1;
			}

			nrf_gpio_pin_write(psel, out_level);
		}

		data->pwm_needed &= ~BIT(channel);
	} else {
		data->pwm_needed |= BIT(channel);
	}

	/* If the PWM generation is not needed for any channel (all are set
	 * to constant inactive or active state), stop the PWM peripheral.
	 * Otherwise, request a playback of the defined sequence so that
	 * the PWM peripheral loads `seq_values` into its internal compare
	 * registers and drives its outputs accordingly.
	 */
	if (data->pwm_needed == 0) {
		/* Don't wait here for the peripheral to actually stop. Instead,
		 * ensure it is stopped before starting the next playback.
		 */
		nrfx_pwm_stop(&config->pwm, false);
		data->stop_requested = true;
	} else {
		if (data->stop_requested) {
			data->stop_requested = false;

			/* After a stop is requested, the PWM peripheral stops
			 * pulse generation at the end of the current period,
			 * and till that moment, it ignores any start requests,
			 * so ensure here that it is stopped.
			 */
			while (!nrfx_pwm_is_stopped(&config->pwm)) {
			}
		}

		/* It is sufficient to play the sequence once without looping.
		 * The PWM generation will continue with the loaded values
		 * until another playback is requested (new values will be
		 * loaded then) or the PWM peripheral is stopped.
		 */
		nrfx_pwm_simple_playback(&config->pwm, &config->seq, 1, 0);
	}

	return 0;
}

int drv_ir_send_on(const struct device *dev)
{
	const struct pwm_nrfx_config *config = dev->config;
	struct pwm_nrfx_data *data = dev->data;

	data->frame.mode = DRV_IR_FRAME_MODE_COOLING;
	data->frame.on = 1;
	data->frame.oscillating = 1;
	data->frame.temperature = 22 - DRV_IR_TEMP_BASE_VALUE;
	data->frame.light = 1;

	data->ext_frame.swing = DRV_IR_EXT_FRAME_SWING_ALL;
	data->ext_frame.temp = DRV_IR_EXT_FRAME_TEMP_SHOW_ALL;
	data->ext_frame.i_feel = 0;
	data->ext_frame.unknown = DRV_IR_EXT_FRAME_UNKNOWN_VAL;
	data->ext_frame.temperature = (
		(22 - DRV_IR_TEMP_BASE_VALUE) + DRV_IR_EXT_FRAME_COOLING_TEMP_OFFSET_C
	) & 0x0F;

	LOG_INF("Frame: %08x", data->frame.content);
	LOG_INF("Extended frame: %08x", data->ext_frame.content);

	config->seq.values.length = drv_ir_encode_frame(
		&data->frame, &data->ext_frame, config->seq.values.p_raw);

	drv_ir_transmit_sequence(dev);

	LOG_DBG("Transmission requested");

	return 0;
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

// static void pwm_handler(nrfx_pwm_evt_type_t event_type, void * p_context)
// // {
// //     nrfx_pwm_t * inst = p_context;
// //     static uint32_t curr_loop = 1;

// //     NRFX_LOG_INFO("Loops: %u / %lu", curr_loop, NUM_OF_LOOPS);

// //     if (curr_loop == NUM_OF_LOOPS)
// //     {
// //         NRFX_LOG_INFO("PWM finished");
// //         nrfx_pwm_uninit(inst);
// //     }
// //     curr_loop++;
// // }

// // static void pwm_handler(nrf_drv_pwm_evt_type_t event)
// {
//     // DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_PWM_INT);

// 	if (((event == NRF_DRV_PWM_EVT_END_SEQ0) || (event == NRF_DRV_PWM_EVT_END_SEQ1)) && (ir_symbol == NULL))
// 	{
// 		nrf_drv_pwm_stop(&pwm, true); // Stop during repetition gap.
// 		// acknowledge_handler(NULL);    // Acknowledge end.
// 		pwm_active = false;

// 		// DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
// 	}
// 	else if ((event == NRF_DRV_PWM_EVT_END_SEQ0) || (event == NRF_DRV_PWM_EVT_END_SEQ1))
// 	{
// 		uint16_t seq_len;

// 		seq_len = nec_repeat_symbol_encoder();
// 		APP_ERROR_CHECK_BOOL(seq_len > 0 && seq_len < MAX_SEQ_SIZE);
// 	}

// 	if (event == NRF_DRV_PWM_EVT_FINISHED)
// 	{
// 		// if (ir_symbol == NULL)
// 		// {
// 	        //     // Acknowledge end.
// 	        //     // acknowledge_handler(NULL);

// 	        //     // DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
// 		// }
// 		// else
// 		// {
// 		// 	__NOP();
// 		// }

// 		pwm_active = false;
// 	}
// }

static nrfx_err_t drv_ir_send_symbol(const sr3_ir_symbol_t *p_ir_symbol)
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
        // acknowledge_handler(NULL);

        // DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_EACK);
	}
	else if (ir_symbol)
	{
		seq_length = frame_encoder(ir_symbol);

		if (seq_length > 0)
		{
			seq.values.p_common   = seq_pwm_values;
			seq.length            = seq_length;
			seq.repeats           = NEC_SYMBOL_REPEATS;
			seq.end_delay         = 0;
		}
		else
		{
			return NRFX_ERROR_NOT_SUPPORTED;
		}

		pwm_active = true;
        // nrf_drv_pwm_simple_playback(&pwm, &seq, NEC_REPETION_MAX,
        //                             NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1);

		nrfx_pwm_simple_playback(&pwm, &seq, 1, NRFX_PWM_FLAG_SIGNAL_END_SEQ0);

        // acknowledge_handler(ir_symbol);
        // DBG_PIN_PULSE(CONFIG_IO_DBG_IR_TX_SACK);
	}

	return NRFX_SUCCESS;
}


// nrfx_err_t drv_ir_enable(void)
// {
//     ASSERT(enabled_flag == false);

//     enabled_flag = true;
//     nrf_pwm_enable(pwm.p_registers);

//     return NRFX_SUCCESS;
// }

// nrfx_err_t drv_ir_disable(void)
// {
//     ASSERT(enabled_flag == true);

//     nrf_pwm_disable(pwm.p_registers);
//     enabled_flag = false;

//     return NRFX_SUCCESS;
// }

static int drv_ir_init(const struct device *dev)
{
	nrfx_err_t status;
	const struct pwm_nrfx_config *config = dev->config;
	struct pwm_nrfx_data *data = dev->data;

	// static const nrf_drv_pwm_config_t config =
	// {
	// 	.output_pins =
	// 	{
	// 		IS_IO_VALID(CONFIG_IO_IR_TX_LED) ? CONFIG_IO_IR_TX_LED : NRF_DRV_PWM_PIN_NOT_USED,
	// 		NRF_DRV_PWM_PIN_NOT_USED,
	// 		NRF_DRV_PWM_PIN_NOT_USED,
	// 		NRF_DRV_PWM_PIN_NOT_USED,
	// 	},

	// 	.irq_priority   = AIRQ_PRIORITY_LOW,
	// 	.base_clock     = NRF_PWM_CLK_16MHz,
	// 	.count_mode     = NRF_PWM_MODE_UP,
	// 	.top_value      = NEC_TOP_VALUE,
	// 	.load_mode      = NRF_PWM_LOAD_COMMON,
	// 	.step_mode      = NRF_PWM_STEP_AUTO
	// };

    // if (acknowledge_handler == NULL)
    // {
    //     return NRF_ERROR_INVALID_PARAM;
    // }

	// acknowledge_handler   = acknowledge_handler;
	enabled_flag          = false;
	pwm_active            = false;
	ir_symbol             = NULL;

    // status = nrf_drv_pwm_init(&pwm, &config, pwm_handler);
    // if (status == NRFX_SUCCESS)
    // {
    //     nrf_pwm_disable(pwm.p_registers);
    // }



#ifdef CONFIG_PINCTRL
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}

	data->initially_inverted = 0;
	for (size_t i = 0; i < ARRAY_SIZE(data->seq_values); i++) {
		uint32_t psel;

		if (channel_psel_get(i, &psel, config)) {
			/* Mark channels as inverted according to what initial
			 * state of their outputs has been set by pinctrl (high
			 * idle state means that the channel is inverted).
			 */
			data->initially_inverted |=
				nrf_gpio_pin_out_read(psel) ? BIT(i) : 0;
		}
	}
#endif

	for (size_t i = 0; i < ARRAY_SIZE(data->seq_values); i++) {
		bool inverted = data->initially_inverted & BIT(i);

		data->seq_values[i] = PWM_NRFX_CH_VALUE(0, inverted);
	}

	







// 	nrfx_pwm_t pwm_instance = NRFX_PWM_INSTANCE(PWM_INST_IDX);
// 	nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG(LED1_PIN, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED);
// 	status = nrfx_pwm_init(&pwm_instance, &config, pwm_handler, &pwm_instance);
// 	NRFX_ASSERT(status == NRFX_SUCCESS);

// #if defined(__ZEPHYR__)
//     #define PWM_INST         NRFX_CONCAT_2(NRF_PWM, PWM_INST_IDX)
//     #define PWM_INST_HANDLER NRFX_CONCAT_3(nrfx_pwm_, PWM_INST_IDX, _irq_handler)
// 	IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(PWM_INST), IRQ_PRIO_LOWEST, PWM_INST_HANDLER, 0);
// #endif

// 	return status;





	nrfx_err_t result = nrfx_pwm_init(&config->pwm,
					  &config->initial_config,
					  NULL,
					  NULL);
	if (result != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize device: %s", dev->name);
		return -EBUSY;
	}

	return 0;
}


#define PWM_NRFX_DEVICE(idx)						      \
	NRF_DT_CHECK_PIN_ASSIGNMENTS(PWM(idx), 1,			      \
				     ch0_pin, ch1_pin, ch2_pin, ch3_pin);     \
	static struct pwm_nrfx_data pwm_nrfx_##idx##_data = {		      \
		COND_CODE_1(CONFIG_PINCTRL, (),				      \
			(.initially_inverted =				      \
				(PWM_CH_INVERTED(idx, 0) ? BIT(0) : 0) |      \
				(PWM_CH_INVERTED(idx, 1) ? BIT(1) : 0) |      \
				(PWM_CH_INVERTED(idx, 2) ? BIT(2) : 0) |      \
				(PWM_CH_INVERTED(idx, 3) ? BIT(3) : 0),))     \
	};								      \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_DEFINE(PWM(idx))));	      \
	static const struct pwm_nrfx_config pwm_nrfx_##idx##_config = {	      \
		.pwm = NRFX_PWM_INSTANCE(idx),				      \
		.initial_config = {					      \
			COND_CODE_1(CONFIG_PINCTRL,			      \
				(.skip_gpio_cfg = true,			      \
				 .skip_psel_cfg = true,),		      \
				(.output_pins = {			      \
					PWM_OUTPUT_PIN(idx, 0),		      \
					PWM_OUTPUT_PIN(idx, 1),		      \
					PWM_OUTPUT_PIN(idx, 2),		      \
					PWM_OUTPUT_PIN(idx, 3),		      \
				 },))					      \
			.base_clock = NRF_PWM_CLK_16MHz,		      \
			.count_mode = (PWM_PROP(idx, center_aligned)	      \
				       ? NRF_PWM_MODE_UP_AND_DOWN	      \
				       : NRF_PWM_MODE_UP),		      \
			.top_value = NEC_TOP_VALUE,			      \
			.load_mode = NRF_PWM_LOAD_COMMON,		      \
			.step_mode = NRF_PWM_STEP_AUTO,		      \
		},							      \
		.seq.values.p_raw = pwm_nrfx_##idx##_data.seq_values,	      \
		.seq.length = NRF_PWM_CHANNEL_COUNT,			      \
		IF_ENABLED(CONFIG_PINCTRL,				      \
			(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(PWM(idx)),))	      \
	};								      \
	PM_DEVICE_DT_DEFINE(PWM(idx), pwm_nrfx_pm_action);		      \
	DEVICE_DT_DEFINE(PWM(idx),					      \
			 drv_ir_init, PM_DEVICE_DT_GET(PWM(idx)),	      \
			 &pwm_nrfx_##idx##_data,			      \
			 &pwm_nrfx_##idx##_config,			      \
			 POST_KERNEL, CONFIG_PWM_INIT_PRIORITY,		      \
			 NULL)

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm0), okay)
PWM_NRFX_DEVICE(0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm1), okay)
PWM_NRFX_DEVICE(1);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm2), okay)
PWM_NRFX_DEVICE(2);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm3), okay)
PWM_NRFX_DEVICE(3);
#endif
