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
#define IFEEL_FRAME_BITS        8

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

// 6 ms high, 3 ms low
const uint16_t IFEEL_START[] =
	{ NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
	  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL, \
	  NEC_MARK_SYMBOL,  NEC_MARK_SYMBOL,					 \
	  NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL, NEC_SPACE_SYMBOL,\
	  NEC_SPACE_SYMBOL };

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

#define FRAME_MODE_OFFSET		0
#define FRAME_ON_OFFSET			3
#define FRAME_FAN_OFFSET		4
#define FRAME_OSCILLATING_OFFSET	6
#define FRAME_SLEEP_OFFSET		7
#define FRAME_SETPOINT_OFFSET	8
#define FRAME_TIMER_OFFSET		12
#define FRAME_TURBO_OFFSET		20
#define FRAME_LIGHT_OFFSET		21
#define FRAME_TREE_OFFSET		22
#define FRAME_XFAN_OFFSET		23
#define FRAME_SCAVENGING_OFFSET		24
#define FRAME_SET_CONFIG_OFFSET		27

#define EXT_FRAME_SWING_OFFSET		0
#define EXT_FRAME_TEMP_SHOW_OFFSET	8
#define EXT_FRAME_I_FEEL_OFFSET		10
#define EXT_FRAME_UNKNOWN_OFFSET	12
#define EXT_FRAME_SETPOINT_OFFSET	28


#define TEMP_BASE_VALUE			16

#define FRAME_MODE_COOLING		0x1
#define FRAME_FAN_AUTO			0x0
#define FRAME_FAN_1			0x1
#define FRAME_FAN_2			0x2
#define FRAME_FAN_3			0x3
#define FRAME_FAN_3			0x3

#define EXT_FRAME_SWING_OFF		0x0
#define EXT_FRAME_SWING_ALL		0x1
#define EXT_FRAME_SWING_FIX_1		0x2 // Highest
#define EXT_FRAME_SWING_FIX_2		0x3 // .
#define EXT_FRAME_SWING_FIX_3		0x4 // .
#define EXT_FRAME_SWING_FIX_4		0x5 // .
#define EXT_FRAME_SWING_FIX_5		0x6 // Lowest
#define EXT_FRAME_SWING_LOW		0x7
#define EXT_FRAME_SWING_MID		0x9
#define EXT_FRAME_SWING_MID		0x9
#define EXT_FRAME_TEMP_SHOW_ALL		0x0
#define EXT_FRAME_TEMP_SET		0x1
#define EXT_FRAME_TEMP_SHOW_IN		0x2
#define EXT_FRAME_TEMP_SHOW_OUT		0x3
#define EXT_FRAME_UNKNOWN_VAL		0x2
#define EXT_FRAME_COOLING_TEMP_OFFSET_C	0x5


struct pwm_nrfx_config {
	nrfx_pwm_t pwm;
	nrfx_pwm_config_t initial_config;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

struct pwm_nrfx_data {
	uint16_t seq_values[MAX_SEQ_SIZE];
	nrf_pwm_sequence_t	 seq;
};


static void insert_start_symbol(uint16_t **seq)
{
	memcpy(*seq, NEC_START, sizeof(NEC_START));
	*seq += ARRAY_SIZE(NEC_START);
}

static void insert_ifeel_start_symbol(uint16_t **seq)
{
	memcpy(*seq, IFEEL_START, sizeof(IFEEL_START));
	*seq += ARRAY_SIZE(IFEEL_START);
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

static void binary_to_symbol(uint32_t val, size_t len, uint16_t **seq)
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

static uint16_t encode_frame(const struct device *dev,
			     const uint32_t *frame,
			     const uint32_t *ext_frame,
			     uint16_t *buf)
{
	uint16_t *seq;
	int i;

	seq = buf;
	insert_start_symbol(&seq);

	binary_to_symbol(*frame, FRAME_BITS, &seq);
	insert_trailing_sequence(&seq);

	insert_ext_frame_space(&seq);

	binary_to_symbol(*ext_frame, EXT_FRAME_BITS, &seq);

	for (i = 0; i < NEC_GUARD_ZEROS; i++)
	{
		insert_zero_symbol(&seq);
	}

	return (uint16_t)(seq - buf);
}

static uint16_t encode_ifeel_frame(const struct device *dev,
				   const uint8_t *ifeel_frame,
				   uint16_t *buf)
{
	uint16_t *seq;
	int i;

	seq = buf;
	insert_ifeel_start_symbol(&seq);

	binary_to_symbol(*ifeel_frame, IFEEL_FRAME_BITS, &seq);
	insert_trailing_sequence(&seq);

	for (i = 0; i < NEC_GUARD_ZEROS; i++)
	{
		insert_zero_symbol(&seq);
	}

	return (uint16_t)(seq - buf);
}

static int transmit_sequence(const struct device *dev)
{
	const struct pwm_nrfx_config *config = dev->config;
	struct pwm_nrfx_data *data = dev->data;

	nrfx_pwm_simple_playback(&config->pwm, &data->seq, 1, 0);

	return 0;
}

static int fill_on(uint32_t *frame,
		   uint32_t *ext_frame,
		   uint8_t temperature_setpoint)
{
	LOG_DBG("temperature setpoint: %d", temperature_setpoint);

	*frame = 0;
	*ext_frame = 0;

	*frame |= FRAME_MODE_COOLING << FRAME_MODE_OFFSET;
	*frame |= 1 << FRAME_ON_OFFSET;
	*frame |= 1 << FRAME_OSCILLATING_OFFSET;
	*frame |= (temperature_setpoint - TEMP_BASE_VALUE)
		 << FRAME_SETPOINT_OFFSET;
	*frame |= 1 << FRAME_LIGHT_OFFSET;

	*ext_frame |= EXT_FRAME_SWING_ALL << EXT_FRAME_SWING_OFFSET;
	*ext_frame |= EXT_FRAME_TEMP_SHOW_ALL << EXT_FRAME_TEMP_SHOW_OFFSET;
	*ext_frame |= 0 << EXT_FRAME_I_FEEL_OFFSET;
	*ext_frame |= EXT_FRAME_UNKNOWN_VAL << EXT_FRAME_UNKNOWN_OFFSET;
	*ext_frame |= (((temperature_setpoint - TEMP_BASE_VALUE)
			+ EXT_FRAME_COOLING_TEMP_OFFSET_C) & 0x0F
		    ) << EXT_FRAME_SETPOINT_OFFSET;

	return 0;
}

static int send_frame(const int32_t *frame,
		      const uint32_t *ext_frame,
		      const struct device *dev)
{
	struct pwm_nrfx_data *data = dev->data;

	LOG_INF("Frame: %08x", *frame);
	LOG_INF("Extended frame: %08x", *ext_frame);

	data->seq.length = encode_frame(
		dev, frame, ext_frame, data->seq_values);

	LOG_INF("Frame length: 0x%x", data->seq.length);
	LOG_INF("Transmission ready");

	transmit_sequence(dev);

	LOG_INF("Transmission requested");

	return 0;
}

int drv_ir_send_on(const struct device *dev, uint8_t temperature_setpoint)
{
	uint32_t frame;
	uint32_t ext_frame;

	fill_on(&frame, &ext_frame, temperature_setpoint);

	send_frame(&frame, &ext_frame, dev);

	return 0;
}

int drv_ir_send_off(const struct device *dev, uint8_t temperature_setpoint)
{
	uint32_t frame;
	uint32_t ext_frame;

	fill_on(&frame, &ext_frame, temperature_setpoint);
	frame &= ~(1 << FRAME_ON_OFFSET);
	ext_frame ^= (BIT(3) << EXT_FRAME_SETPOINT_OFFSET);

	send_frame(&frame, &ext_frame, dev);

	return 0;
}

int drv_ir_send_change_config(const struct device *dev, uint8_t temperature_setpoint)
{
	uint32_t frame;
	uint32_t ext_frame;

	fill_on(&frame, &ext_frame, temperature_setpoint);
	frame |= 1 << FRAME_SET_CONFIG_OFFSET;

	send_frame(&frame, &ext_frame, dev);

	return 0;
}

int drv_ir_send_ifeel(const struct device *dev, uint8_t current_temp)
{
	struct pwm_nrfx_data *data = dev->data;
	uint8_t ifeel_frame = 0;

	LOG_INF("Current temperature: %d", current_temp);

	ifeel_frame = current_temp;

	LOG_INF("Ifeel frame: %08x", ifeel_frame);

	data->seq.length = encode_ifeel_frame(
		dev, &ifeel_frame, data->seq_values);

	LOG_INF("Frame length: 0x%x", data->seq.length);
	LOG_INF("Transmission ready");

	transmit_sequence(dev);

	LOG_INF("Transmission requested");

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
