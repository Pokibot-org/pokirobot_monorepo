#define DT_DRV_COMPAT zephyr_uart_bus

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#include <pokibot/drivers/uart_bus.h>

LOG_MODULE_REGISTER(uart_bus, CONFIG_UART_BUS_LOG_LEVEL);

#define UART_BUS_MAX_BUFFER 64

struct uart_bus_config {
    const struct device *uart_dev;
};

struct uart_bus_buffers {
    uint8_t tx[UART_BUS_MAX_BUFFER];
    uint8_t rx[UART_BUS_MAX_BUFFER];
};

struct uart_bus_data {
    struct uart_bus_buffers *buffers;
    struct k_mutex access_mutex;
    struct k_sem rx_ready;
#ifdef CONFIG_UART_BUS_LOG_LEVEL_DBG
    int64_t t_tx_done;
    int64_t t_rx_ready;
#endif
};

void uart_bus_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
    const struct device *shared_dev = user_data;
    struct uart_bus_data *data = shared_dev->data;

    switch (evt->type) {
        case UART_TX_DONE:
            IF_ENABLED(CONFIG_UART_BUS_LOG_LEVEL_DBG, (data->t_tx_done = k_uptime_get()));
            break;
        case UART_RX_RDY:
            IF_ENABLED(CONFIG_UART_BUS_LOG_LEVEL_DBG, (data->t_rx_ready = k_uptime_get()));
            k_sem_give(&data->rx_ready);
            break;
        case UART_RX_STOPPED:
            k_sem_reset(&data->rx_ready);
            break;
        default:
            /* ignore */
            break;
    }
}

int uart_bus_send_receive(const struct device *dev, const uint8_t *tx_data, size_t tx_len,
                          size_t rx_len, uint8_t *rx_buffer, k_timeout_t timeout)
{
    const struct uart_bus_config *cfg = dev->config;
    struct uart_bus_data *data = dev->data;
    int ret;
#ifdef CONFIG_UART_BUS_LOG_LEVEL_DBG
    int64_t t_start;
#endif

    if (tx_len + rx_len > UART_BUS_MAX_BUFFER) {
        return -ENOMEM;
    }

    k_timepoint_t expiry = sys_timepoint_calc(timeout);

    k_mutex_lock(&data->access_mutex, sys_timepoint_timeout(expiry));
    k_sem_reset(&data->rx_ready);

    memcpy(data->buffers->tx, tx_data, tx_len);

    /* clear all the received bytes in case the uart driver implement a fifo and
    we received unexpected bytes. */
    uint8_t none;
    while (!uart_poll_in(cfg->uart_dev, &none)) {
    }

    ret = uart_rx_enable(cfg->uart_dev, data->buffers->rx, tx_len + rx_len, SYS_FOREVER_US);
    if (ret < 0) {
        LOG_DBG("failed to setup rx: %d", ret);
        goto cleanup0;
    }

#ifdef CONFIG_UART_BUS_LOG_LEVEL_DBG
    t_start = k_uptime_get();
#endif

    ret = uart_tx(cfg->uart_dev, data->buffers->tx, tx_len, SYS_FOREVER_US);
    if (ret < 0) {
        LOG_DBG("failed to setup tx: %d", ret);
        goto cleanup1;
    }

    ret = k_sem_take(&data->rx_ready, sys_timepoint_timeout(expiry));
    if (ret < 0) {
        LOG_DBG("rx failed: %d", ret);
        goto cleanup1;
    }

    /* update the rx buffer pointer to the start of rx_data */
    memcpy(rx_buffer, &data->buffers->rx[tx_len], rx_len);

#ifdef CONFIG_UART_BUS_LOG_LEVEL_DBG
    const int d_tx = (int)(data->t_tx_done - t_start);
    const int d_rx = rx_len > 0 ? (int)(data->t_rx_ready - data->t_tx_done) : 0;

    if ((d_tx > 1) || (d_rx > 1)) {
        LOG_WRN("d_tx=%dms, d_rx=%dms", d_tx, d_rx);
        LOG_HEXDUMP_WRN(*rx_buffer, tx_len + rx_len, "rx_frame=");
    }
#endif

cleanup1:
    uart_rx_disable(cfg->uart_dev);
    uart_tx_abort(cfg->uart_dev);
cleanup0:
    k_mutex_unlock(&data->access_mutex);

    return ret;
}

static int uart_bus_init(const struct device *dev)
{
    int ret;
    const struct uart_bus_config *cfg = dev->config;
    struct uart_bus_data *data = dev->data;

    if (!device_is_ready(cfg->uart_dev)) {
        return -ENODEV;
    }

    ret = k_mutex_init(&data->access_mutex);
    if (ret < 0) {
        LOG_DBG("mutex init failed: %d", ret);
        return ret;
    }

    ret = k_sem_init(&data->rx_ready, 0, 1);
    if (ret < 0) {
        LOG_DBG("rx sem init failed: %d", ret);
        return ret;
    }

    ret = uart_callback_set(cfg->uart_dev, uart_bus_callback, (void *)dev);
    if (ret < 0) {
        LOG_DBG("setting uart callback failed: %d", ret);
        return ret;
    }

    return 0;
}

#define UART_BUS_INIT(inst)                                                                        \
    static struct uart_bus_buffers __nocache buffers_##inst;                                       \
    static const struct uart_bus_config uart_bus_cfg_##inst = {                                    \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                              \
    };                                                                                             \
    static struct uart_bus_data uart_bus_data_##inst = {.buffers = &buffers_##inst};                \
    DEVICE_DT_INST_DEFINE(inst, &uart_bus_init, NULL, &uart_bus_data_##inst, &uart_bus_cfg_##inst, \
                          POST_KERNEL, CONFIG_UART_BUS_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(UART_BUS_INIT)
