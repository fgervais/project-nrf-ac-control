#ifndef MQTT_H_
#define MQTT_H_

struct mqtt_subscription {
	const char *topic;
	void (*callback)(const char *);
};

int mqtt_publish_to_topic(const char *topic, char *payload, bool retain);
int mqtt_subscribe_to_topic(const struct mqtt_subscription *subs,
			    size_t nb_of_subs);
int mqtt_init(const char *dev_id);

#endif /* MQTT_H_ */