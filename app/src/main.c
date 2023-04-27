#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include <app_event_manager.h>

#define MODULE main
#include <caf/events/module_state_event.h>
#include <caf/events/button_event.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include "drv_ir.h"
#include "ha.h"
#include "openthread.h"


#define TMP116_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_tmp116)

#define CHANGE_MODE_EVENT_OFF		BIT(0)
#define CHANGE_MODE_EVENT_COOL		BIT(1)
#define CHANGE_SETPOINT_EVENT		BIT(2)
#define CHANGE_STATE_EVENT		BIT(8)


static double temperature_setpoint = -1;
static bool current_state_off = true;
static bool enabled;

static K_EVENT_DEFINE(ac_control_events);

static void mode_change_callback(const char *mode)
{
	if (strcmp(mode, "cool") == 0) {
		LOG_DBG("❄️  mode %s", mode);
		k_event_post(&ac_control_events, CHANGE_MODE_EVENT_COOL);
	}
	else if (strcmp(mode, "off") == 0) {
		LOG_DBG("🔌 mode %s", mode);
		k_event_post(&ac_control_events, CHANGE_MODE_EVENT_OFF);
	}
}

static void temperature_setpoint_change_callback(double setpoint)
{
	LOG_DBG("🌡️  setpoint: %g°C", setpoint);
	temperature_setpoint = setpoint;
	k_event_post(&ac_control_events, CHANGE_SETPOINT_EVENT);
}

double get_current_temperature(const struct device *const dev)
{
	int ret;
	struct sensor_value temp_value;

	ret = sensor_sample_fetch(dev);
	if (ret) {
		LOG_ERR("Failed to fetch measurements (%d)", ret);
		return 0;
	}

	ret = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP,
				 &temp_value);
	if (ret) {
		LOG_ERR("Failed to get measurements (%d)", ret);
		return 0;
	}

	return sensor_value_to_double(&temp_value);
}

bool get_current_device_state(const struct device *port, gpio_pin_t pin)
{
	return (bool)gpio_pin_get_raw(port, pin);
}

void main(void)
{
	double current_temp;
	uint32_t events;

	const struct device *pwm0 = DEVICE_DT_GET(DT_NODELABEL(pwm0));
	const struct device *gpios[] = {
		DEVICE_DT_GET(DT_NODELABEL(gpio0)),
		DEVICE_DT_GET(DT_NODELABEL(gpio1)),
	};
	const struct device *const tmp117 = DEVICE_DT_GET(TMP116_NODE);


	LOG_INF("\n\n👟 MAIN START 👟\n");

	if (app_event_manager_init()) {
		LOG_ERR("Event manager not initialized");
	} else {
		module_set_state(MODULE_STATE_READY);
	}

	__ASSERT(device_is_ready(tmp117), "TMP117 device not ready");
	LOG_INF("Device %s - %p is ready", tmp117->name, tmp117);

	openthread_enable_ready_flag();

	while (!openthread_ready)
		k_sleep(K_MSEC(100));

	// Something else is not ready, not sure what
	k_sleep(K_MSEC(100));

	ha_start(mode_change_callback, temperature_setpoint_change_callback);

	enabled = get_current_device_state(gpios[CONFIG_APP_STATE_INPUT_PORT],
					   CONFIG_APP_STATE_INPUT_PIN);
	k_event_post(&ac_control_events, CHANGE_STATE_EVENT);

	LOG_INF("┌──────────────────────────────────────────────────────────┐");
	LOG_INF("│ Entering main loop                                       │");
	LOG_INF("└──────────────────────────────────────────────────────────┘");

	k_timeout_t timeout;

	while (1) {
		if (current_state_off) {
			timeout = K_FOREVER;
		}
		else {
			timeout = K_MINUTES(3);
		}


		events = k_event_wait(&ac_control_events,
			     (CHANGE_STATE_EVENT |
			      CHANGE_SETPOINT_EVENT |
			      CHANGE_MODE_EVENT_COOL |
			      CHANGE_MODE_EVENT_OFF),
			     false, timeout);

		k_event_set(&ac_control_events, 0);

		if (events == 0) {
			LOG_INF("📡 broadcast current temperature");

			current_temp = get_current_temperature(tmp117);
			LOG_INF("🌡️  current temp: %g°C", current_temp);

			ha_send_current_temp(current_temp);
			drv_ir_send_ifeel(pwm0, current_temp);
			continue;
		}

		if (current_state_off && (events & CHANGE_MODE_EVENT_COOL)) {
			LOG_INF("📡 turn ON");
			drv_ir_send_on(pwm0, temperature_setpoint);
			k_sleep(K_SECONDS(1));

			current_temp = get_current_temperature(tmp117);
			LOG_INF("🌡️  current temp: %g°C", current_temp);

			ha_send_current_temp(current_temp);
			drv_ir_send_ifeel(pwm0, current_temp);
			current_state_off = false;
		}
		else if (!current_state_off && (events & CHANGE_MODE_EVENT_OFF)) {
			LOG_INF("📡 turn OFF");
			drv_ir_send_off(pwm0, temperature_setpoint);
			current_state_off = true;
		}

		if (events & CHANGE_SETPOINT_EVENT) {
			LOG_INF("📡 change setpoint: %g°C", temperature_setpoint);
			drv_ir_send_change_config(pwm0, temperature_setpoint);
		}

		if (events & CHANGE_STATE_EVENT) {
			if (enabled) {
				LOG_DBG("✅ enabled");
			}
			else {
				LOG_DBG("❌ disabled");
			}
			ha_send_current_state(enabled);
		}
	}
}

static bool event_handler(const struct app_event_header *eh)
{
	const struct button_event *evt;

	if (is_button_event(eh)) {
		evt = cast_button_event(eh);

		if (!evt->pressed) {
			enabled = true;
		}
		else {
			enabled = false;
		}

		k_event_post(&ac_control_events, CHANGE_STATE_EVENT);
	}

	return true;
}

APP_EVENT_LISTENER(MODULE, event_handler);
APP_EVENT_SUBSCRIBE(MODULE, button_event);
