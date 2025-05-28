#include "pomicontrol.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pomicontrol);

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(rf_module))
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(rf_module));

/*
static const struct gpio_dt_spec rf_pin_set = GPIO_DT_SPEC_GET(DT_NODELABEL(rf_pin_set), gpios);
void pomicontrol_send_frame(char *cmd)
{
    LOG_INF("> %s", cmd);
    for (int i = 0; i < strlen(cmd); i ++ ) {
        uart_poll_out(dev, cmd[i]);
    }
    uart_poll_out(dev, '\n');
    k_sleep(K_MSEC(200));
}

void pomicontrol_uart_clbk(const struct device *dev, void *user_data)
{
    static char buffer[128];
    static int index = 0;

    if(uart_irq_rx_ready(dev))
    {
        char c;
        uart_fifo_read(dev, &c, 1);
        if(c == '\r') {
            return;
        }
        if (index == sizeof(buffer)) {
            index = 0;
        }
        buffer[index++] = c;

        if (c == '\n') {
            buffer[index] = 0;
            LOG_INF("< %s", buffer);
            index = 0;
        }
    }
}

void pomicontrol_configure(void) {
    gpio_pin_configure_dt(&rf_pin_set, GPIO_OUTPUT_LOW);
    uart_irq_callback_user_data_set(dev, pomicontrol_uart_clbk, NULL);
    uart_irq_rx_enable(dev);
    k_sleep(K_MSEC(100));

    pomicontrol_send_frame("AT+V");
    pomicontrol_send_frame("AT+C066");
    pomicontrol_send_frame("AT+FU3");
    pomicontrol_send_frame("AT+RX");

    gpio_pin_set_dt(&rf_pin_set, 1);
    k_sleep(K_MSEC(100));
}
*/

int pomicontrol_init(void)
{
    struct uart_config cfg = {
        .baudrate = 1200,
        .data_bits = UART_CFG_DATA_BITS_8,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        .parity = UART_CFG_PARITY_NONE,
    };
    uart_configure(dev, &cfg);
    // pomicontrol_configure();
    return 0;
}

SYS_INIT(pomicontrol_init, APPLICATION, 0);


void pomicontrol_activate(void)
{
    static uint16_t counter = 0;
    uint8_t frame[32];
    sprintf(frame, "POKIBOT GO %d\n", counter++);
    for (int i = 0; i < strlen(frame); i++) {
        uart_poll_out(dev, frame[i]);
    }
}
#else
void pomicontrol_activate(void){}
#endif
