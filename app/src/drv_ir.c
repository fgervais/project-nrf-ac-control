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
 *  * Copyright (c) 2018, Cue Health Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nrfx_pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(drv_ir, LOG_LEVEL_DBG);


#define NEC_GUARD_ZEROS         1
#define TRAILING_SEQUENCE_BITS  8
#define FRAME_BITS              28
#define EXT_FRAME_BITS          32

#define NEC_TOP_VALUE           421      // 38kHz carrier 26.3125 usec for 16MHz PWM_CLK
#define NEC_SYMBOL_REPEATS      21       // repeat each symbol 21 times - 21*26.3125=552.5625 us

#define NEC_MARK_SYMBOL         0x808C   // 140/421 (1/16000000 sec)  - duty 1/3
#define NEC_SPACE_SYMBOL        0x8000


// 9 ms high, 4.4 ms low
const uint16_t NEC_START[] =
	{ NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
	  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
	  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
	  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
	  NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
	  NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL};

// 1010 0101
const uint16_t TRAILING_SEQUENCE[] =
	{ NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL , NEC_SPACE_SYMBOL, \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL,					  \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL , NEC_SPACE_SYMBOL, \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL,					  \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL,					  \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL , NEC_SPACE_SYMBOL, \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL,					  \
	  NEC_MARK_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL , NEC_SPACE_SYMBOL };

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
	  NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL };

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
#define DRV_IR_EXT_FRAME_SWING_FIX_1	0x2 // Highest
#define DRV_IR_EXT_FRAME_SWING_FIX_2	0x3 // .
#define DRV_IR_EXT_FRAME_SWING_FIX_3	0x4 // .
#define DRV_IR_EXT_FRAME_SWING_FIX_4	0x5 // .
#define DRV_IR_EXT_FRAME_SWING_FIX_5	0x6 // Lowest
#define DRV_IR_EXT_FRAME_SWING_LOW	0x7
#define DRV_IR_EXT_FRAME_SWING_MID	0x9
#define DRV_IR_EXT_FRAME_SWING_MID	0x9
#define DRV_IR_EXT_FRAME_TEMP_SHOW_ALL	0x0
#define DRV_IR_EXT_FRAME_TEMP_SET	0x1
#define DRV_IR_EXT_FRAME_TEMP_SHOW_IN	0x2
#define DRV_IR_EXT_FRAME_TEMP_SHOW_OUT	0x3
#define DRV_IR_EXT_FRAME_UNKNOWN_VAL	0x2
#define DRV_IR_EXT_FRAME_COOLING_TEMP_OFFSET_C	0x5


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
	};
};

union drv_ir_ext_frame
{
	uint32_t content;
	struct {
		uint32_t swing 		: 4;
		uint32_t 		: 4;
		uint32_t temp 		: 2;
		uint32_t i_feel 	: 1;
		uint32_t 		: 1;
		uint32_t unknown 	: 4;
		uint32_t 		: 12;
		uint32_t temperature 	: 4;
	};
};

struct pwm_nrfx_config {
	nrfx_pwm_t pwm;
	nrfx_pwm_config_t initial_config;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

struct pwm_nrfx_data {
	uint16_t seq_values[MAX_SEQ_SIZE];
	nrf_pwm_sequence_t	seq;
	union drv_ir_frame	frame;
	union drv_ir_ext_frame	ext_frame;
};


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

static void drv_ir_binary_to_symbol(uint32_t val, size_t len, uint16_t **seq)
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

static uint16_t drv_ir_encode_frame(const struct device *dev,
				    const union drv_ir_frame *frame,
				    const union drv_ir_ext_frame *ext_frame,
				    const uint16_t *buf)
{
	uint16_t *seq;
	int i;

	struct pwm_nrfx_data *data = dev->data;

	seq = data->seq_values;
	insert_start_symbol(&seq);

	drv_ir_binary_to_symbol(data->frame.content, FRAME_BITS, &seq);
	insert_trailing_sequence(&seq);

	insert_ext_frame_space(&seq);

	drv_ir_binary_to_symbol(data->ext_frame.content, EXT_FRAME_BITS, &seq);

	for (i = 0; i < NEC_GUARD_ZEROS; i++)
	{
		insert_zero_symbol(&seq);
	}

	return (uint16_t)(seq - data->seq_values);
}

static int drv_ir_transmit_sequence(const struct device *dev)
{
	const struct pwm_nrfx_config *config = dev->config;
	struct pwm_nrfx_data *data = dev->data;

	nrfx_pwm_simple_playback(&config->pwm, &data->seq, 1, 0);

	return 0;
}

#define TARGET_TEMPERATURE 22
int drv_ir_send_on(const struct device *dev)
{
	struct pwm_nrfx_data *data = dev->data;

	data->frame.mode = DRV_IR_FRAME_MODE_COOLING;
	data->frame.on = 1;
	data->frame.oscillating = 1;
	data->frame.temperature = TARGET_TEMPERATURE - DRV_IR_TEMP_BASE_VALUE;
	data->frame.light = 1;

	data->ext_frame.swing = DRV_IR_EXT_FRAME_SWING_ALL;
	data->ext_frame.temp = DRV_IR_EXT_FRAME_TEMP_SHOW_ALL;
	data->ext_frame.i_feel = 0;
	data->ext_frame.unknown = DRV_IR_EXT_FRAME_UNKNOWN_VAL;
	data->ext_frame.temperature = (
		(TARGET_TEMPERATURE - DRV_IR_TEMP_BASE_VALUE)
			+ DRV_IR_EXT_FRAME_COOLING_TEMP_OFFSET_C
	) & 0x0F;

	LOG_INF("Frame: %08x", data->frame.content);
	LOG_INF("Extended frame: %08x", data->ext_frame.content);

	data->seq.length = drv_ir_encode_frame(
		dev, &data->frame, &data->ext_frame, data->seq_values);

	LOG_INF("Frame length: 0x%x", data->seq.length);
	LOG_INF("Transmission ready");

	drv_ir_transmit_sequence(dev);

	LOG_DBG("Transmission requested");

	return 0;
}

static int drv_ir_init(const struct device *dev)
{
	const struct pwm_nrfx_config *config = dev->config;

	LOG_INF("Init");

#ifdef CONFIG_PINCTRL
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}
#endif

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

#ifdef CONFIG_PM_DEVICE
static void drv_ir_uninit(const struct device *dev)
{
	const struct pwm_nrfx_config *config = dev->config;

	nrfx_pwm_uninit(&config->pwm);

	memset(dev->data, 0, sizeof(struct pwm_nrfx_data));
}

static int drv_ir_pm_action(const struct device *dev,
			      enum pm_device_action action)
{
#ifdef CONFIG_PINCTRL
	const struct pwm_nrfx_config *config = dev->config;
#endif
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
#ifdef CONFIG_PINCTRL
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
#endif
		ret = drv_ir_init(dev);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		drv_ir_uninit(dev);

#ifdef CONFIG_PINCTRL
		ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if (ret < 0) {
			return ret;
		}
#endif
		break;

	default:
		return -ENOTSUP;
	}

	return ret;
}
#else

#define drv_ir_pm_action NULL

#endif /* CONFIG_PM_DEVICE */

#define PWM(dev_idx) DT_NODELABEL(pwm##dev_idx)
#define PWM_PROP(dev_idx, prop) DT_PROP(PWM(dev_idx), prop)

#define PWM_CH_INVERTED(dev_idx, ch_idx) \
	PWM_PROP(dev_idx, ch##ch_idx##_inverted)

#define PWM_OUTPUT_PIN(dev_idx, ch_idx)					\
	COND_CODE_1(DT_NODE_HAS_PROP(PWM(dev_idx), ch##ch_idx##_pin),	\
		(PWM_PROP(dev_idx, ch##ch_idx##_pin) |			\
			(PWM_CH_INVERTED(dev_idx, ch_idx)		\
			 ? NRFX_PWM_PIN_INVERTED : 0)),			\
		(NRFX_PWM_PIN_NOT_USED))

#define PWM_NRFX_DEVICE(idx)						      \
	NRF_DT_CHECK_PIN_ASSIGNMENTS(PWM(idx), 1,			      \
				     ch0_pin, ch1_pin, ch2_pin, ch3_pin);     \
	static struct pwm_nrfx_data pwm_nrfx_##idx##_data = {		      \
		.seq.values.p_raw = pwm_nrfx_##idx##_data.seq_values,	      \
		.seq.repeats = NEC_SYMBOL_REPEATS,			      \
	};								      \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_DEFINE(PWM(idx))));	      \
	static struct pwm_nrfx_config pwm_nrfx_##idx##_config = {	      \
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
			.step_mode = NRF_PWM_STEP_AUTO,			      \
		},							      \
		IF_ENABLED(CONFIG_PINCTRL,				      \
			(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(PWM(idx)),))	      \
	};								      \
	PM_DEVICE_DT_DEFINE(PWM(idx), drv_ir_pm_action);		      \
	DEVICE_DT_DEFINE(PWM(idx),					      \
			 drv_ir_init, PM_DEVICE_DT_GET(PWM(idx)),	      \
			 &pwm_nrfx_##idx##_data,			      \
			 &pwm_nrfx_##idx##_config,			      \
			 POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,     \
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
