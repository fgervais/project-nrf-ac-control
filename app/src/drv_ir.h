#ifndef DRV_IR_H_
#define DRV_IR_H_

int drv_ir_send_on(const struct device *dev, uint8_t temperature_setpoint);
int drv_ir_send_ifeel(const struct device *dev, uint8_t current_temp);
int drv_ir_send_off(const struct device *dev);
int drv_ir_send_change_config(const struct device *dev, uint8_t temperature_setpoint);

#endif /* DRV_IR_H_ */