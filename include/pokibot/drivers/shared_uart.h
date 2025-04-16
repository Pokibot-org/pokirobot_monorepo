#ifndef ZEPHYR_INCLUDE_SHARED_UART_H_
#define ZEPHYR_INCLUDE_SHARED_UART_H_
#include <zephyr/kernel.h>
#include <zephyr/device.h>

int shared_uart_send_receive(const struct device *dev, const uint8_t *tx_data, size_t tx_len,
							 size_t rx_len, uint8_t **rx_buffer, k_timeout_t timeout);

#endif
