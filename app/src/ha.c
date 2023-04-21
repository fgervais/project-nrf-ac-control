#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(home_assistant, LOG_LEVEL_DBG);

#include <zephyr/data/json.h>
#include <zephyr/drivers/hwinfo.h>

#include <stdio.h>

#include "mqtt.h"


#define DEVICE_ID_BYTE_SIZE	8


struct device {
	const char *identifiers;
	const char *name;
	const char *sw_version;
	const char *hw_version;
	const char *model;
	const char *manufacturer;
};

struct config {
	const char *base_path;
	const char *name;
	int initial;
	int min_temp;
	int max_temp;
	const char *modes[2];
	int number_of_modes;
	const char *mode_command_topic;
	const char *mode_command_template;
	const char *temperature_command_topic;
	int precision;
	struct device dev;
};


static char device_id[DEVICE_ID_BYTE_SIZE * 2 + 1];

static struct config ac_config = {
	.base_path = "home/room/kitchen/climate/ac",
	.name = "Split Air Conditioner",
	.initial = 22,
	.min_temp = 16,
	.max_temp = 30,
	.modes = { "off", "cool" },
	.number_of_modes = 2,
	.mode_command_topic = "~/mode/set",
	.mode_command_template = "{{ value if value=='off' else 'on' }}",
	.temperature_command_topic = "~/temperature/set",
	.precision = 1,
	.dev = {
		.identifiers = device_id,
		.name = "Split Air Conditioner",
		.sw_version = "7cdc3fb1ddce7a31e6192e2760791e24a2d11d41",
		.hw_version = "1.0.0",
		.model = "unknown",
		.manufacturer = "GREE ELECTRIC APPLIANCES INC.",	
	},
};

static const struct json_obj_descr device_descr[] = {
	JSON_OBJ_DESCR_PRIM(struct device, identifiers,	 JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct device, name,	 JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct device, sw_version,	 JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct device, hw_version,	 JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct device, model,	 JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct device, manufacturer, JSON_TOK_STRING),
};

static const struct json_obj_descr config_descr[] = {
	JSON_OBJ_DESCR_PRIM_NAMED(struct config, "~", base_path,	JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct config, name,			JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct config, initial,			JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct config, min_temp,			JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_PRIM(struct config, max_temp,			JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_ARRAY(struct config, modes, 2, number_of_modes,	JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct config, mode_command_topic,		JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct config, mode_command_template,	JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct config, temperature_command_topic,	JSON_TOK_STRING),
	JSON_OBJ_DESCR_PRIM(struct config, precision,			JSON_TOK_NUMBER),
	JSON_OBJ_DESCR_OBJECT(struct config, dev, device_descr),
};


static int get_device_id_string(char *id_string, size_t id_string_len)
{
	uint8_t dev_id[DEVICE_ID_BYTE_SIZE];
	ssize_t length;
	int ret;

	length = hwinfo_get_device_id(dev_id, sizeof(dev_id));

	if (length == -ENOTSUP) {
		LOG_ERR("Not supported by hardware");
		return -ENOTSUP;
	} else if (length < 0) {
		LOG_ERR("Error: %zd", length);
		return length;
	}

	ret = bin2hex(dev_id, ARRAY_SIZE(dev_id), id_string, id_string_len);
	LOG_DBG("bin2hex(): %d", ret);

	return 0;
}

static int ha_subscribe_to_topics(void)
{
	return 0;
}

// <discovery_prefix>/<component>/[<node_id>/]<object_id>/config
//
// Best practice for entities with a unique_id is to set <object_id> to
// unique_id and omit the <node_id>.
// https://www.home-assistant.io/integrations/mqtt/#discovery-topic
// #define DISCOVERY_TOPIC_FORMAT_STRING	"homeassistant/climate/%s/config"
#define DISCOVERY_TOPIC_FORMAT_STRING	"test/climate/%s/config"
static int ha_send_discovery(void)
{
	int ret;
	char json_config[512];
	char discovery_topic[sizeof(DISCOVERY_TOPIC_FORMAT_STRING) - 2
			     + DEVICE_ID_BYTE_SIZE * 2];

	snprintf(discovery_topic, sizeof(discovery_topic),
		 DISCOVERY_TOPIC_FORMAT_STRING, ac_config.dev.identifiers);

	ret = json_obj_encode_buf(config_descr, ARRAY_SIZE(config_descr),
				  &ac_config, json_config, sizeof(json_config));
	if (ret < 0) {
		LOG_ERR("Could not encode JSON (%d)", ret);
		return ret;
	}

	LOG_DBG("payload: %s", json_config);
	return 0;

	mqtt_publish_discovery(json_config, discovery_topic);

	return 0;
}

int ha_send_current_temp(double current_temp)
{
	return 0;
}

// int ha_start(mode_change_callback, temperature_setpoint_change_callback)
int ha_start(void)
{
	int ret;

	ret = get_device_id_string(
		device_id,
		ARRAY_SIZE(device_id));
	if (ret < 0) {
		LOG_ERR("Could not get device ID");
		return ret;
	}

	LOG_INF("Device ID: 0x%s", ac_config.dev.identifiers);

	ha_send_discovery();
	// ha_subscribe_to_topics();

	return 0;
}
