#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mqtt, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/rand32.h>

#include <assert.h>
#include <errno.h>
#include <string.h>

#include "mqtt.h"


#define MQTT_BUFFER_SIZE	1024


static uint8_t rx_buffer[MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[MQTT_BUFFER_SIZE];

static struct mqtt_client client_ctx;

static struct sockaddr_storage broker;

/* Socket Poll */
static struct zsock_pollfd fds[1];
static int nfds;

static const char *device_id;
static bool mqtt_connected;

static const struct mqtt_subscription *subscriptions;
static size_t number_of_subscriptions;

static struct k_work_delayable keepalive_work;

#if defined(CONFIG_DNS_RESOLVER)
static struct zsock_addrinfo hints;
static struct zsock_addrinfo *haddr;
#endif

static const struct device *wdt;
static int wdt_channel_id;


static void mqtt_event_handler(struct mqtt_client *const client,
			       const struct mqtt_evt *evt);
static void keepalive(struct k_work *work);


static void prepare_fds(struct mqtt_client *client)
{
	fds[0].events = ZSOCK_POLLIN;
	nfds = 1;
}

static void clear_fds(void)
{
	nfds = 0;
}

static int wait(int timeout)
{
	int rc = -EINVAL;

	if (nfds <= 0) {
		return rc;
	}

	rc = zsock_poll(fds, nfds, timeout);
	if (rc < 0) {
		LOG_ERR("poll error: %d", errno);
		return -errno;
	}

	return rc;
}

static void broker_init(void)
{
	struct sockaddr_in6 *broker6 = (struct sockaddr_in6 *)&broker;

	broker6->sin6_family = AF_INET6;
	broker6->sin6_port = htons(CONFIG_APP_MQTT_SERVER_PORT);

#if defined(CONFIG_DNS_RESOLVER)
	net_ipaddr_copy(&broker6->sin6_addr,
			&net_sin6(haddr->ai_addr)->sin6_addr);
#else
	zsock_inet_pton(AF_INET6, CONFIG_APP_MQTT_SERVER_ADDR, &broker6->sin_addr);
#endif

	k_work_init_delayable(&keepalive_work, keepalive);
}

static void client_init(struct mqtt_client *client)
{
	mqtt_client_init(client);

	broker_init();

	client->broker = &broker;
	client->evt_cb = mqtt_event_handler;

	client->client_id.utf8 = (uint8_t *)device_id;
	client->client_id.size = strlen(device_id);

	LOG_DBG("client id: %s", device_id);

	client->password = NULL;
	client->user_name = NULL;

	client->protocol_version = MQTT_VERSION_3_1_1;

	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	client->transport.type = MQTT_TRANSPORT_NON_SECURE;
}

static void mqtt_event_handler(struct mqtt_client *const client,
			       const struct mqtt_evt *evt)
{
	uint8_t data[33];
	int len;
	int bytes_read;

	switch (evt->type) {
	case MQTT_EVT_SUBACK:
		LOG_INF("SUBACK packet id: %u", evt->param.suback.message_id);
		break;

	case MQTT_EVT_UNSUBACK:
		LOG_INF("UNSUBACK packet id: %u", evt->param.suback.message_id);
		break;

	case MQTT_EVT_CONNACK:
		if (evt->result) {
			LOG_ERR("MQTT connect failed %d", evt->result);
			break;
		}

		mqtt_connected = true;
		LOG_DBG("MQTT client connected!");
		break;

	case MQTT_EVT_DISCONNECT:
		LOG_DBG("MQTT client disconnected %d", evt->result);

		mqtt_connected = false;
		clear_fds();
		break;

	case MQTT_EVT_PUBACK:
		if (evt->result) {
			LOG_ERR("MQTT PUBACK error %d", evt->result);
			break;
		}

		LOG_INF("PUBACK packet id: %u", evt->param.puback.message_id);
		break;

	case MQTT_EVT_PUBLISH:
		len = evt->param.publish.message.payload.len;

		LOG_INF("📨 MQTT publish received %d, %d bytes", evt->result, len);
		LOG_INF("   ├── id: %d, qos: %d", evt->param.publish.message_id,
			evt->param.publish.message.topic.qos);
		LOG_INF("   ├── topic: %.*s",
			evt->param.publish.message.topic.topic.size,
			evt->param.publish.message.topic.topic.utf8);

		while (len) {
			bytes_read = mqtt_read_publish_payload(&client_ctx,
					data,
					len >= sizeof(data) - 1 ?
					sizeof(data) - 1 : len);
			if (bytes_read < 0 && bytes_read != -EAGAIN) {
				LOG_ERR("failure to read payload");
				break;
			}

			data[bytes_read] = '\0';
			LOG_INF("   └── payload: %s", data);
			len -= bytes_read;
		}

		for (int i = 0; i < number_of_subscriptions; i++) {
			if (strncmp(subscriptions[i].topic,
				    evt->param.publish.message.topic.topic.utf8,
				    evt->param.publish.message.topic.topic.size) == 0) {
				subscriptions[i].callback(data);
				break;
			}
		}
		break;

	case MQTT_EVT_PINGRESP:
		LOG_DBG("PINGRESP");
		LOG_DBG("🦴 feed watchdog");
		wdt_feed(wdt, wdt_channel_id);
		break;

	default:
		LOG_DBG("Unhandled MQTT event %d", evt->type);
		break;
	}
}

static void subscribe(struct mqtt_client *client, const char *topic)
{
	int err;
	struct mqtt_topic subs_topic;
	struct mqtt_subscription_list subs_list;

	subs_topic.topic.utf8 = topic;
	subs_topic.topic.size = strlen(topic);
	subs_topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
	subs_list.list = &subs_topic;
	subs_list.list_count = 1U;
	subs_list.message_id = 1U;

	err = mqtt_subscribe(client, &subs_list);
	if (err) {
		LOG_ERR("Failed on topic %s", topic);
	}
}

static void keepalive(struct k_work *work)
{
	int rc;

	LOG_DBG("🤖 mqtt keepalive");

	if (!mqtt_connected) {
		LOG_WRN("we are disconnected");
		return;
	}

	if (client_ctx.unacked_ping) {
		LOG_DBG("🤔 MQTT ping not acknowledged: %d",
			client_ctx.unacked_ping);
	}

	rc = mqtt_ping(&client_ctx);
	if (rc < 0) {
		LOG_ERR("mqtt_ping (%d)", rc);
		return;
	}

	k_work_reschedule(&keepalive_work, K_SECONDS(client_ctx.keepalive));
}

static void poll_thread_function(void)
{
	int rc;

	while (1) {
		if (wait(SYS_FOREVER_MS)) {
			rc = mqtt_input(&client_ctx);
			if (rc < 0) {
				LOG_ERR("mqtt_input (%d)", rc);
				return;
			}
		}
	}
}

K_THREAD_DEFINE(poll_thread, CONFIG_APP_POLL_THREAD_STACK_SIZE,
		poll_thread_function, NULL, NULL, NULL,
		K_LOWEST_APPLICATION_THREAD_PRIO, 0, SYS_FOREVER_MS);


#define MQTT_CONNECT_TIMEOUT_MS	1000
#define MQTT_ABORT_TIMEOUT_MS	5000
static int try_to_connect(struct mqtt_client *client)
{
	uint8_t retries = 3U;
	int rc;

	LOG_DBG("attempting to connect...");

	while (retries--) {
		client_init(client);

		rc = mqtt_connect(client);
		if (rc) {
			LOG_ERR("mqtt_connect failed %d", rc);
			continue;
		}

		prepare_fds(client);

		rc = wait(MQTT_CONNECT_TIMEOUT_MS);
		if (rc < 0) {
			mqtt_abort(client);
			return rc;
		}

		mqtt_input(client);

		if (mqtt_connected) {
			k_work_reschedule(&keepalive_work,
					  K_SECONDS(client->keepalive));
			k_thread_start(poll_thread);
			return 0;
		}

		mqtt_abort(client);

		wait(MQTT_ABORT_TIMEOUT_MS);
	}

	return -EINVAL;
}

#if defined(CONFIG_DNS_RESOLVER)
static int get_mqtt_broker_addrinfo(void)
{
	int retries = 3;
	int rc = -EINVAL;

	while (retries--) {
		hints.ai_family = AF_INET6;
		hints.ai_socktype = SOCK_STREAM;
		hints.ai_protocol = 0;

		rc = getaddrinfo(CONFIG_APP_MQTT_SERVER_HOSTNAME,
				       STRINGIFY(CONFIG_APP_MQTT_SERVER_PORT),
				       &hints, &haddr);
		if (rc == 0) {
			char atxt[INET6_ADDRSTRLEN] = { 0 };

			LOG_INF("DNS resolved for %s:%d",
			CONFIG_APP_MQTT_SERVER_HOSTNAME,
			CONFIG_APP_MQTT_SERVER_PORT);

			assert(haddr->ai_addr->sa_family == AF_INET6);

			inet_ntop(
			    AF_INET6,
			    &((const struct sockaddr_in6 *)haddr->ai_addr)->sin6_addr,
			    atxt, sizeof(atxt));

			LOG_INF("address: %s", atxt);

			return 0;
		}

		LOG_ERR("DNS not resolved for %s:%d, retrying",
			CONFIG_APP_MQTT_SERVER_HOSTNAME,
			CONFIG_APP_MQTT_SERVER_PORT);

		k_sleep(K_MSEC(200));
	}

	return rc;
}
#endif

static int connect_to_server(void)
{
	int rc = -EINVAL;

#if defined(CONFIG_DNS_RESOLVER)
	rc = get_mqtt_broker_addrinfo();
	if (rc) {
		return rc;
	}
#endif

	rc = try_to_connect(&client_ctx);
	if (rc) {
		return rc;
	}

	LOG_INF("mqtt keepalive: %ds", client_ctx.keepalive);

	return 0;
}

static int watchdog_init(void)
{
	int err;
	
	wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));

	if (!device_is_ready(wdt)) {
		LOG_ERR("%s: device not ready", wdt->name);
		return -ENODEV;
	}

	struct wdt_timeout_cfg wdt_config = {
		.window.min = 0,
		.window.max = (3 * 60 + 40) * MSEC_PER_SEC,
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC,
	};

	wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
	if (wdt_channel_id < 0) {
		LOG_ERR("watchdog install error");
		return wdt_channel_id;
	}

	err = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err < 0) {
		LOG_ERR("watchdog setup error");
		return 0;
	}

	LOG_DBG("🐶 watchdog started!");

	return 0;
}

int mqtt_publish_to_topic(const char *topic, char *payload, bool retain)
{
	int ret;
	struct mqtt_publish_param param;

	LOG_DBG("📤 %s", topic);
	LOG_DBG("   └── payload: %s", payload);

	param.message.topic.qos = MQTT_QOS_0_AT_MOST_ONCE;
	param.message.topic.topic.utf8 = (uint8_t *)topic;
	param.message.topic.topic.size = strlen(topic);
	param.message.payload.data = payload;
	param.message.payload.len = strlen(payload);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = retain ? 1U : 0U;

	if (!mqtt_connected) {
		ret = connect_to_server();
		if (ret < 0) {
			return ret;
		}
	}

	ret = mqtt_publish(&client_ctx, &param);
	if (ret < 0) {
		LOG_ERR("mqtt_publish (%d)", ret);
		return ret;
	}

	return 0;
}

int mqtt_subscribe_to_topic(const struct mqtt_subscription *subs,
			    size_t nb_of_subs)
{
	int ret;

	if (!mqtt_connected) {
		ret = connect_to_server();
		if (ret < 0) {
			return ret;
		}
	}

	subscriptions = subs;
	number_of_subscriptions = nb_of_subs;

	for (int i = 0; i < nb_of_subs; i++) {
		LOG_INF("subscribing to topic: %s", subs[i].topic);
		subscribe(&client_ctx, subs[i].topic);		
	}

	return 0;
}

int mqtt_init(const char *dev_id)
{
	int ret;

	device_id = dev_id;

	ret = watchdog_init();
	if (ret < 0) {
		LOG_ERR("could not init watchdog");
		return ret;
	}

	return 0;
}
