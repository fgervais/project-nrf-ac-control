#ifndef HA_H_
#define HA_H_

int ha_start(void (*mode_change_cb)(const char *mode),
	     void (*temperature_setpoint_change_cb)(double setpoint));
int ha_send_current_temp(double current_temp);

#endif /* HA_H_ */