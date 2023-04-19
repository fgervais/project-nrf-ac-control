
// {
//   "~": "home/room/kitchen/climate/ac",
//   "name": "Split Air Conditioner",
//   "initial": 22,
//   "min_temp": 16,
//   "max_temp": 30,

//   "modes": "['off', 'cool']",
//   "mode_command_topic": "~/mode/set",
//   "mode_command_template": "{{ value if value=='off' else 'on' }}",
//   "temperature_command_topic": "~/temperature/set",
//   "precision": 1.0,

//   "device": {
//     "identifiers": cmd_get_device_id(),
//     "name": "Split Air Conditioner",
//     "sw_version": "7cdc3fb1ddce7a31e6192e2760791e24a2d11d41",
//     "hw_version": "1.0.0", 
//     "model": "unknown",
//     "manufacturer": "GREE ELECTRIC APPLIANCES INC."
//   }
// }


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(home_assistant, LOG_LEVEL_DBG);

#include <zephyr/drivers/hwinfo.h>


#define DEVICE_ID_BYTE_SIZE 8
static int get_device_id(uint64_t *device_id)
{
	uint8_t dev_id[DEVICE_ID_BYTE_SIZE];
	ssize_t length;
	int i;

	length = hwinfo_get_device_id(dev_id, sizeof(dev_id));

	if (length == -ENOTSUP) {
		LOG_ERR("Not supported by hardware");
		return -ENOTSUP;
	} else if (length < 0) {
		LOG_ERR("Error: %zd", length);
		return length;
	}

	LOG_INF("Length: %zd\n", length);
	LOG_INF("ID: 0x");

	memcpy(device_id, dev_id, DEVICE_ID_BYTE_SIZE);

	return 0;
}

static int ha_send_discovery(void)
{

}

static int ha_subscribe_to_topics(void)
{

}

int ha_send_current_temp(double current_temp)
{

}

int ha_start(mode_change_callback, temperature_setpoint_change_callback)
{
	ha_send_discovery()
	ha_subscribe_to_topics();
}
