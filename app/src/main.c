#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include "drv_ir.h"
#include "ha.h"
#include "openthread.h"


#define TMP116_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(ti_tmp116)

#define CHANGE_MODE_EVENT_OFF		BIT(0)
#define CHANGE_MODE_EVENT_COOL		BIT(1)


static double temperature_setpoint = -1;

static K_EVENT_DEFINE(change_mode_event);

static void mode_change_callback(const char *mode)
{
	// if (temperature_setpoint < 0) {
	// 	LOG_ERR("setpoint not set");
	// 	return;
	// }

	if (strcmp(mode, "cool") == 0) {
		LOG_DBG("❄️  mode %s", mode);
		k_event_post(&change_mode_event, CHANGE_MODE_EVENT_COOL);
		// drv_ir_send_on(pwm0, temperature_setpoint);
		// k_sleep(K_SECONDS(1));
		// drv_ir_send_ifeel(pwm0, current_temp);
	}
	else if (strcmp(mode, "off") == 0) {
		LOG_DBG("🔌 mode %s", mode);
		k_event_post(&change_mode_event, CHANGE_MODE_EVENT_OFF);
		// drv_ir_send_on(pwm0, temperature_setpoint);
	}
}

static void temperature_setpoint_change_callback(double setpoint)
{
	LOG_DBG("🌡️  setpoint: %f", setpoint);
	temperature_setpoint = setpoint;
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

void main(void)
{
	double current_temp;

	const struct device *pwm0 = DEVICE_DT_GET(DT_NODELABEL(pwm0));
	const struct device *const tmp117 = DEVICE_DT_GET(TMP116_NODE);


	LOG_INF("\n\n👟 MAIN START 👟\n");

	__ASSERT(device_is_ready(tmp117), "TMP117 device not ready");
	LOG_INF("Device %s - %p is ready", tmp117->name, tmp117);

	// while(1) {
		// LOG_INF("────────────────────────────────────────");
		// current_temp = get_current_temperature(tmp117);

		// LOG_INF("🌡️ temp is %g°C", current_temp);

		// drv_ir_send_on(pwm0);
		// k_sleep(K_SECONDS(1));
		// drv_ir_send_ifeel(pwm0, current_temp);
		// k_sleep(K_SECONDS(5));
	// }

	openthread_enable_ready_flag();

	while (!openthread_ready)
		k_sleep(K_MSEC(100));

	// Something else is not ready, not sure what
	k_sleep(K_MSEC(100));

	ha_start(mode_change_callback, temperature_setpoint_change_callback);

	// while (1) {
	// 	k_event_wait(&change_mode_event,
	// 		     CHANGE_MODE_EVENT_COOL | CHANGE_MODE_EVENT_OFF,
	// 		     false, K_FOREVER);
	// }

	LOG_INF("****************************************");
	LOG_INF("MAIN DONE");
	LOG_INF("****************************************");
}
