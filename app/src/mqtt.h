#ifndef MQTT_H_
#define MQTT_H_

int mqtt_publish_discovery(char *json_config, const char *topic);
int mqtt_subscribe_to_topic(const char *topic);
int mqtt_init(const char *dev_id);

#endif /* MQTT_H_ */