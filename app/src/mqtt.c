#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mqtt, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/socket.h>
#include <zephyr/random/rand32.h>

#include <assert.h>
#include <errno.h>
#include <string.h>


#define MQTT_BUFFER_SIZE	1024
#define MQTT_CLIENTID 		"zephyr_publisher"


/* Buffers for MQTT client. */
static uint8_t rx_buffer[MQTT_BUFFER_SIZE];
static uint8_t tx_buffer[MQTT_BUFFER_SIZE];

/* The mqtt client struct */
static struct mqtt_client client_ctx;

/* MQTT Broker details. */
static struct sockaddr_storage broker;

/* Socket Poll */
static struct zsock_pollfd fds[1];
static int nfds;

static bool mqtt_connected;

// static struct k_work_delayable pub_message;
static struct k_work_delayable keepalive_work;

#if defined(CONFIG_DNS_RESOLVER)
static struct zsock_addrinfo hints;
static struct zsock_addrinfo *haddr;
#endif

// static K_SEM_DEFINE(mqtt_start, 0, 1);

/* Application TLS configuration details */
// #define TLS_SNI_HOSTNAME CONFIG_SAMPLE_CLOUD_AZURE_HOSTNAME
// #define APP_CA_CERT_TAG 1

// static sec_tag_t m_sec_tags[] = {
// 	APP_CA_CERT_TAG,
// };

// static uint8_t topic[] = "devices/" MQTT_CLIENTID "/messages/devicebound/#";
static struct mqtt_topic subs_topic;
static struct mqtt_subscription_list subs_list;

static void mqtt_event_handler(struct mqtt_client *const client,
			       const struct mqtt_evt *evt);
static void keepalive(struct k_work *work);
// static int poll_mqtt(void);

// static int tls_init(void)
// {
// 	int err;

// 	err = tls_credential_add(APP_CA_CERT_TAG, TLS_CREDENTIAL_CA_CERTIFICATE,
// 				 ca_certificate, sizeof(ca_certificate));
// 	if (err < 0) {
// 		LOG_ERR("Failed to register public certificate: %d", err);
// 		return err;
// 	}

// 	return err;
// }

static void prepare_fds(struct mqtt_client *client)
{
	// if (client->transport.type == MQTT_TRANSPORT_SECURE) {
	// 	fds[0].fd = client->transport.tls.sock;
	// }

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
	// static struct mqtt_utf8 password;
	// static struct mqtt_utf8 username;
	// struct mqtt_sec_config *tls_config;

	mqtt_client_init(client);

	broker_init();

	/* MQTT client configuration */
	client->broker = &broker;
	client->evt_cb = mqtt_event_handler;

	client->client_id.utf8 = (uint8_t *)MQTT_CLIENTID;
	client->client_id.size = strlen(MQTT_CLIENTID);

	client->password = NULL;
	client->user_name = NULL;

	// password.utf8 = (uint8_t *)CONFIG_SAMPLE_CLOUD_AZURE_PASSWORD;
	// password.size = strlen(CONFIG_SAMPLE_CLOUD_AZURE_PASSWORD);

	// client->password = &password;

	// username.utf8 = (uint8_t *)CONFIG_SAMPLE_CLOUD_AZURE_USERNAME;
	// username.size = strlen(CONFIG_SAMPLE_CLOUD_AZURE_USERNAME);

	// client->user_name = &username;

	client->protocol_version = MQTT_VERSION_3_1_1;

	/* MQTT buffers configuration */
	client->rx_buf = rx_buffer;
	client->rx_buf_size = sizeof(rx_buffer);
	client->tx_buf = tx_buffer;
	client->tx_buf_size = sizeof(tx_buffer);

	/* MQTT transport configuration */
	client->transport.type = MQTT_TRANSPORT_NON_SECURE;

	// tls_config = &client->transport.tls.config;

	// tls_config->peer_verify = TLS_PEER_VERIFY_REQUIRED;
	// tls_config->cipher_list = NULL;
	// tls_config->sec_tag_list = m_sec_tags;
	// tls_config->sec_tag_count = ARRAY_SIZE(m_sec_tags);
	// tls_config->hostname = TLS_SNI_HOSTNAME;

// #if defined(CONFIG_SOCKS)
// 	mqtt_client_set_proxy(client, &socks5_proxy,
// 			      socks5_proxy.sa_family == AF_INET ?
// 			      sizeof(struct sockaddr_in) :
// 			      sizeof(struct sockaddr_in6));
// #endif
}

static void mqtt_event_handler(struct mqtt_client *const client,
			       const struct mqtt_evt *evt)
{
	struct mqtt_puback_param puback;
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

		LOG_INF("MQTT publish received %d, %d bytes", evt->result, len);
		LOG_INF(" id: %d, qos: %d", evt->param.publish.message_id,
			evt->param.publish.message.topic.qos);
		LOG_INF(" topic: %s", evt->param.publish.message.topic.topic.utf8);

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
			LOG_INF("   payload: %s", data);
			len -= bytes_read;
		}

		puback.message_id = evt->param.publish.message_id;
		mqtt_publish_qos1_ack(&client_ctx, &puback);
		break;

	case MQTT_EVT_PINGRESP:
		LOG_DBG("PINGRESP");
		break;

	default:
		LOG_DBG("Unhandled MQTT event %d", evt->type);
		break;
	}
}

static void subscribe(struct mqtt_client *client, const char *topic)
{
	int err;

	subs_topic.topic.utf8 = topic;
	subs_topic.topic.size = strlen(topic);
	subs_list.list = &subs_topic;
	subs_list.list_count = 1U;
	subs_list.message_id = 1U;

	err = mqtt_subscribe(client, &subs_list);
	if (err) {
		LOG_ERR("Failed on topic %s", topic);
	}

	// poll_mqtt();
}

// static int publish(struct mqtt_client *client, enum mqtt_qos qos)
// {
// 	char payload[] = "{id=123}";
// 	char topic[] = "devices/" MQTT_CLIENTID "/messages/events/";
// 	uint8_t len = strlen(topic);
// 	struct mqtt_publish_param param;

// 	param.message.topic.qos = qos;
// 	param.message.topic.topic.utf8 = (uint8_t *)topic;
// 	param.message.topic.topic.size = len;
// 	param.message.payload.data = payload;
// 	param.message.payload.len = strlen(payload);
// 	param.message_id = sys_rand32_get();
// 	param.dup_flag = 0U;
// 	param.retain_flag = 0U;

// 	return mqtt_publish(client, &param);
// }

// static int poll_mqtt(void)
// {
// 	int rc;

// 	if (!mqtt_connected) {
// 		LOG_WRN("not connected");
// 		return -ENOTCONN;
// 	}

// 	if (wait(1000)) {
// 		rc = mqtt_input(&client_ctx);
// 		if (rc < 0) {
// 			LOG_ERR("mqtt_input (%d)", rc);
// 			return rc;
// 		}
// 	}

// 	return 0;

// 	// while (mqtt_connected) {
// 	// 	rc = wait(SYS_FOREVER_MS);
// 	// 	if (rc > 0) {
// 	// 		mqtt_input(&client_ctx);
// 	// 	}
// 	// }
// }

/* Random time between 10 - 15 seconds
 * If you prefer to have this value more than CONFIG_MQTT_KEEPALIVE,
 * then keep the application connection live by calling mqtt_live()
 * in regular intervals.
 */
// static uint8_t timeout_for_publish(void)
// {
// 	return (10 + sys_rand32_get() % 5);
// }

// static void publish_timeout(struct k_work *work)
// {
// 	int rc;

// 	if (!mqtt_connected) {
// 		return;
// 	}

// 	rc = publish(&client_ctx, MQTT_QOS_1_AT_LEAST_ONCE);
// 	if (rc) {
// 		LOG_ERR("mqtt_publish ERROR");
// 		goto end;
// 	}

// 	LOG_DBG("mqtt_publish OK");
// end:
// 	k_work_reschedule(&pub_message, K_SECONDS(timeout_for_publish()));
// }

static void keepalive(struct k_work *work)
{
	int rc;

	LOG_DBG("🤖 mqtt keepalive");

	if (!mqtt_connected) {
		LOG_WRN("we are disconnected");
		return;
	}

	if (client_ctx.unacked_ping) {
		LOG_DBG("Previous MQTT ping not acknowledged");
		// return -ECONNRESET;
		return;
	}

	rc = mqtt_ping(&client_ctx);
	if (rc < 0) {
		LOG_ERR("mqtt_ping (%d)", rc);
		return;
	}

	// rc = poll_mqtt();
	// if (rc < 0) {
	// 	LOG_ERR("poll_mqtt (%d)", rc);
	// 	return;
	// }

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








// 	int err;
// 	struct pollfd fds[1];
// 	struct aws_iot_evt aws_iot_evt = {
// 		.type = AWS_IOT_EVT_DISCONNECTED,
// 		.data = { .err = AWS_IOT_DISCONNECT_MISC}
// 	};

// start:
// 	k_sem_take(&connection_poll_sem, K_FOREVER);
// 	atomic_set(&connection_poll_active, 1);

// 	aws_iot_evt.data.err = AWS_IOT_CONNECT_RES_SUCCESS;
// 	aws_iot_evt.type = AWS_IOT_EVT_CONNECTING;
// 	aws_iot_notify_event(&aws_iot_evt);

// 	err = connect_client(NULL);
// 	if (err != AWS_IOT_CONNECT_RES_SUCCESS) {
// 		aws_iot_evt.data.err = err;
// 		aws_iot_evt.type = AWS_IOT_EVT_CONNECTING;
// 		aws_iot_notify_event(&aws_iot_evt);
// 		goto reset;
// 	}

// 	LOG_DBG("AWS IoT broker connection request sent");

// 	fds[0].fd = client.transport.tls.sock;
// 	fds[0].events = POLLIN;

// 	aws_iot_evt.type = AWS_IOT_EVT_DISCONNECTED;
// 	atomic_set(&aws_iot_disconnected, 0);

// 	while (true) {
// 		err = poll(fds, ARRAY_SIZE(fds), aws_iot_keepalive_time_left());

// 		/* If poll returns 0 the timeout has expired. */
// 		if (err == 0) {
// 			err = aws_iot_ping();
// 			if (err) {
// 				LOG_ERR("Cloud MQTT keepalive ping failed: %d", err);
// 				aws_iot_evt.data.err = AWS_IOT_DISCONNECT_MISC;
// 				break;
// 			}
// 			continue;
// 		}

// 		if ((fds[0].revents & POLLIN) == POLLIN) {
// 			err = aws_iot_input();
// 			if (err) {
// 				LOG_ERR("Cloud MQTT input error: %d", err);
// 				if (err == -ENOTCONN) {
// 					break;
// 				}
// 			}

// 			if (atomic_get(&aws_iot_disconnected) == 1) {
// 				LOG_DBG("The cloud socket is already closed.");
// 				break;
// 			}

// 			continue;
// 		}

// 		if (err < 0) {
// 			LOG_ERR("poll() returned an error: %d", err);
// 			aws_iot_evt.data.err = AWS_IOT_DISCONNECT_MISC;
// 			break;
// 		}

// 		if ((fds[0].revents & POLLNVAL) == POLLNVAL) {
// 			LOG_DBG("Socket error: POLLNVAL");
// 			LOG_DBG("The cloud socket was unexpectedly closed.");
// 			aws_iot_evt.data.err =
// 					AWS_IOT_DISCONNECT_INVALID_REQUEST;
// 			break;
// 		}

// 		if ((fds[0].revents & POLLHUP) == POLLHUP) {
// 			LOG_DBG("Socket error: POLLHUP");
// 			LOG_DBG("Connection was closed by the cloud.");
// 			aws_iot_evt.data.err =
// 					AWS_IOT_DISCONNECT_CLOSED_BY_REMOTE;
// 			break;
// 		}

// 		if ((fds[0].revents & POLLERR) == POLLERR) {
// 			LOG_DBG("Socket error: POLLERR");
// 			LOG_DBG("Cloud connection was unexpectedly closed.");
// 			aws_iot_evt.data.err = AWS_IOT_DISCONNECT_MISC;
// 			break;
// 		}
// 	}

// 	/* Upon a socket error, disconnect the client and notify the
// 	 * application. If the client has already been disconnected this has
// 	 * occurred via a MQTT DISCONNECT event and the application has
// 	 * already been notified.
// 	 */
// 	if (atomic_get(&aws_iot_disconnected) == 0) {
// 		aws_iot_notify_event(&aws_iot_evt);
// 		aws_iot_disconnect();
// 	}

// reset:
// 	atomic_set(&connection_poll_active, 0);
// 	k_sem_take(&connection_poll_sem, K_NO_WAIT);
// 	goto start;
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
			// subscribe(client);
			// k_work_reschedule(&pub_message,
			// 		  K_SECONDS(timeout_for_publish()));
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

// static void connect_to_cloud_and_publish(void)
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

/* DHCP tries to renew the address after interface is down and up.
 * If DHCPv4 address renewal is success, then it doesn't generate
 * any event. We have to monitor this way.
 * If DHCPv4 attempts exceeds maximum number, it will delete iface
 * address and attempts for new request. In this case we can rely
 * on IPV4_ADDR_ADD event.
 */
// #if defined(CONFIG_NET_DHCPV4)
// static void check_network_connection(struct k_work *work)
// {
// 	struct net_if *iface;

// 	if (mqtt_connected) {
// 		return;
// 	}

// 	iface = net_if_get_default();
// 	if (!iface) {
// 		goto end;
// 	}

// 	if (iface->config.dhcpv4.state == NET_DHCPV4_BOUND) {
// 		k_sem_give(&mqtt_start);
// 		return;
// 	}

// 	LOG_INF("waiting for DHCP to acquire addr");

// end:
// 	k_work_reschedule(&check_network_conn, K_SECONDS(3));
// }
// #endif

// #if defined(CONFIG_NET_DHCPV4)
// static void abort_mqtt_connection(void)
// {
// 	if (mqtt_connected) {
// 		mqtt_connected = false;
// 		mqtt_abort(&client_ctx);
// 		k_work_cancel_delayable(&pub_message);
// 	}
// }

// static void l4_event_handler(struct net_mgmt_event_callback *cb,
// 			     uint32_t mgmt_event, struct net_if *iface)
// {
// 	if ((mgmt_event & L4_EVENT_MASK) != mgmt_event) {
// 		return;
// 	}

// 	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
// 		/* Wait for DHCP to be back in BOUND state */
// 		k_work_reschedule(&check_network_conn, K_SECONDS(3));

// 		return;
// 	}

// 	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
// 		abort_mqtt_connection();
// 		k_work_cancel_delayable(&check_network_conn);

// 		return;
// 	}
// }
// #endif

// int main(void)
// {
// 	int rc;

// 	LOG_DBG("Waiting for network to setup...");

// 	// rc = tls_init();
// 	// if (rc) {
// 	// 	return 0;
// 	// }

// 	// k_work_init_delayable(&pub_message, publish_timeout);

// // #if defined(CONFIG_NET_DHCPV4)
// // 	k_work_init_delayable(&check_network_conn, check_network_connection);

// // 	net_mgmt_init_event_callback(&l4_mgmt_cb, l4_event_handler,
// // 				     L4_EVENT_MASK);
// // 	net_mgmt_add_event_callback(&l4_mgmt_cb);
// // #endif

// 	// connect_to_cloud_and_publish();
// 	return 0;
// }

int mqtt_publish_current_temp(double current_temp)
{
	return 0;
}

int mqtt_publish_discovery(char *json_config, const char *topic)
{
	// char payload[] = "{id=123}";
	// char topic[] = "devices/" MQTT_CLIENTID "/messages/events/";
	// uint8_t len = strlen(topic);
	int ret;
	struct mqtt_publish_param param;

	param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
	param.message.topic.topic.utf8 = (uint8_t *)topic;
	param.message.topic.topic.size = strlen(topic);
	param.message.payload.data = json_config;
	param.message.payload.len = strlen(json_config);
	param.message_id = sys_rand32_get();
	param.dup_flag = 0U;
	param.retain_flag = 0U;

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

	// ret = poll_mqtt();
	// if (ret < 0) {
	// 	LOG_ERR("poll_mqtt (%d)", ret);
	// 	return ret;
	// }

	return 0;
}

int mqtt_subscribe_to_topic(const char *topic)
{
	int ret;

	LOG_INF("subscribing to topic: %s", topic);

	if (!mqtt_connected) {
		ret = connect_to_server();
		if (ret < 0) {
			return ret;
		}
	}

	subscribe(&client_ctx, topic);

	return 0;
}
