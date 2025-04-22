#ifndef ZEPHYR_DRIVERS_POKSTEPPER_ADI_TMC_UART_H_
#define ZEPHYR_DRIVERS_POKSTEPPER_ADI_TMC_UART_H_
#include <zephyr/device.h>

int tmc_reg_read(const struct device *uart_bus_dev, uint8_t slave_address, uint8_t reg,
                 uint32_t *data);
int tmc_reg_write(const struct device *uart_bus_dev, uint8_t slave_address, uint8_t reg,
                  uint32_t value);
#endif
