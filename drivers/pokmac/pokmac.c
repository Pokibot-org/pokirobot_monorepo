#include <stddef.h>
#include <stdint.h>
#define DT_DRV_COMPAT pokibot_pokmac
#include <pokibot/drivers/pokmac.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/crc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(pokmac_driver, CONFIG_POKMAC_LOG_LEVEL);

#define POKMAC_SYNC_SIZE     2
#define POKMAC_HEADER_SIZE   3
#define POKMAC_CRC_SIZE      2
#define POKMAC_RESERVED_SIZE (POKMAC_SYNC_SIZE + POKMAC_HEADER_SIZE + POKMAC_CRC_SIZE)

#define POKMAC_RX_EVENT_RECEIVED_ACK BIT(1)

enum pokmac_frame_type {
    POKMAC_FRAME_TYPE_NONE = 0,
    POKMAC_FRAME_TYPE_WRITE_NO_ACK = 1,
    POKMAC_FRAME_TYPE_WRITE_WITH_ACK = 2,
    POKMAC_FRAME_TYPE_ACK = 3,
    POKMAC_FRAME_TYPE_NOACK = 4,
    POKMAC_FRAME_TYPE_LAST = 5,
};

struct pokmac_cfg {
    const struct device *uart_dev;
};

struct pokmac_data {
    const struct device *self;
    struct k_mutex uart_mutex;
    size_t receive_index;
    size_t packet_completed_index;
    bool receiving;
    uint8_t receive_buffer[CONFIG_POKMAC_BUFFER_SIZE];
    uint8_t send_buffer[CONFIG_POKMAC_BUFFER_SIZE];
    sys_slist_t recv_clbks;
    struct k_work send_ack_work;
    struct k_event rx_events;
};

static void send_frame(const struct device *dev, const uint8_t *buffer, size_t buffer_size)
{
    const struct pokmac_cfg *cfg = dev->config;
    for (size_t i = 0; i < buffer_size; i++) {
        uart_poll_out(cfg->uart_dev, buffer[i]);
    }
}

static void send_ack_work_handler(struct k_work *work)
{
    struct pokmac_data *data = CONTAINER_OF(work, struct pokmac_data, send_ack_work);
    const struct device *dev = data->self;
    LOG_DBG("Sending ACK");
    const uint8_t buffer[] = {0xDE, 0xAD, POKMAC_FRAME_TYPE_ACK, 0, 0, 0x81, 0xC0};
    k_mutex_lock(&data->uart_mutex, K_FOREVER);
    send_frame(dev, buffer, sizeof(buffer));
    k_mutex_unlock(&data->uart_mutex);
}

static int register_recv_callback(const struct device *dev, struct pokmac_recv_callback *clbk)
{
    struct pokmac_data *data = dev->data;
    sys_slist_append(&data->recv_clbks, &clbk->node);
    return 0;
}

static int send(const struct device *dev, uint8_t *payload_buffer, size_t payload_size,
                bool confirmed)
{
    struct pokmac_data *data = dev->data;

    const size_t frame_size = payload_size + POKMAC_RESERVED_SIZE;
    if (frame_size > CONFIG_POKMAC_BUFFER_SIZE) {
        return -ENOMEM;
    }
    // SYNC
    data->send_buffer[0] = 0xDE;
    data->send_buffer[1] = 0xAD;
    // HEADER
    data->send_buffer[2] =
        confirmed ? POKMAC_FRAME_TYPE_WRITE_WITH_ACK : POKMAC_FRAME_TYPE_WRITE_NO_ACK;
    data->send_buffer[3] = payload_size >> 8;
    data->send_buffer[4] = payload_size & 0xFFU;
    // APP
    memcpy(&data->send_buffer[5], payload_buffer, payload_size);
    // CRC
    size_t crc_index = frame_size - POKMAC_CRC_SIZE;
    uint16_t crc = crc16_ansi(&data->send_buffer[POKMAC_SYNC_SIZE],
                              frame_size - POKMAC_CRC_SIZE - POKMAC_SYNC_SIZE);
    data->send_buffer[crc_index] = crc;
    data->send_buffer[crc_index + 1] = crc >> 8;

    LOG_DBG("Sending msg, payload size %d, crc %X", payload_size, crc);
    // SENDING
    int ret = 0;
    k_mutex_lock(&data->uart_mutex, K_FOREVER);
    k_event_clear(&data->rx_events, 0xFFFFFFFF);
    send_frame(dev, data->send_buffer, frame_size);
    if (confirmed) {
        LOG_DBG("Confirmed msg, waiting for ack");
        if (k_event_wait(&data->rx_events, POKMAC_RX_EVENT_RECEIVED_ACK, false,
                         K_MSEC(CONFIG_POKMAC_ACK_TIMEOUT_MS)) & POKMAC_RX_EVENT_RECEIVED_ACK) {
            ret = 0;
        } else {
            LOG_DBG("Did not receive an ACK");
            ret = -ETIMEDOUT;
        }
    }
    k_mutex_unlock(&data->uart_mutex);
    return ret;
}

void pokprotocol_decode_frame(struct pokmac_data *obj)
{
    uint8_t frame_type = obj->receive_buffer[0];
    uint16_t payload_size = (uint16_t)obj->receive_buffer[1] << 8 | obj->receive_buffer[2];
    if (payload_size > CONFIG_POKMAC_BUFFER_SIZE - POKMAC_RESERVED_SIZE) {
        LOG_ERR("Payload size too big");
        return;
    }
    uint8_t *payload = &obj->receive_buffer[3];
    size_t crc_start = POKMAC_HEADER_SIZE + payload_size;
    uint16_t recv_crc = (uint16_t)obj->receive_buffer[crc_start] |
                        (uint16_t)obj->receive_buffer[crc_start + 1] << 8;
    uint16_t crc = crc16_ansi(obj->receive_buffer, obj->receive_index - POKMAC_CRC_SIZE);
    LOG_DBG("Received msg, payload size %d, recv_crc %X | crc %X", payload_size, recv_crc, crc);

    if (crc != recv_crc) {
        LOG_ERR("Wrong crc");
        // TODO: send noack
        return;
    }

    if (frame_type == POKMAC_FRAME_TYPE_ACK) {
        LOG_DBG("Received ACK");
        k_event_set(&obj->rx_events, POKMAC_RX_EVENT_RECEIVED_ACK);
        return;
    } else if (frame_type == POKMAC_FRAME_TYPE_WRITE_WITH_ACK) {
        k_work_submit(&obj->send_ack_work);
    }

    struct pokmac_recv_callback *clbk;
    SYS_SLIST_FOR_EACH_CONTAINER(&obj->recv_clbks, clbk, node) {
        if (clbk->cb) {
            clbk->cb(payload, payload_size);
        }
    }
}

int pokprotocol_feed_byte(struct pokmac_data *obj, uint8_t byte)
{
    if (!obj->receiving) {
        // SYNCING on the 0xDEAD word
        if (byte == 0xDE && obj->receive_index == 0) {
            obj->receive_index++;
        } else if (byte == 0xAD && obj->receive_index == 1) {
            obj->receive_index = 0;
            obj->receiving = true;
        }
    } else {
        if (obj->receive_index >= CONFIG_POKMAC_BUFFER_SIZE) {
            obj->receive_index = 0;
            obj->receiving = false;
            return -1;
        }
        obj->receive_buffer[obj->receive_index] = byte;
        obj->receive_index++;

        if (obj->receive_index == POKMAC_HEADER_SIZE) {
            uint16_t payload_size = (uint16_t)obj->receive_buffer[1] << 8 | obj->receive_buffer[2];
            obj->packet_completed_index = POKMAC_HEADER_SIZE + payload_size + POKMAC_CRC_SIZE;
        }
        if (obj->receive_index >= POKMAC_HEADER_SIZE &&
            obj->packet_completed_index == obj->receive_index) {
            pokprotocol_decode_frame(obj);
            obj->receive_index = 0;
            obj->receiving = false;
        }
    }


    return 0;
}

static void pokmac_uart_isr(const struct device *uart, void *user_data)
{
    const struct device *dev = user_data;
    const struct pokmac_cfg *cfg = dev->config;
    struct pokmac_data *data = dev->data;

    while (uart_irq_update(cfg->uart_dev) > 0 && uart_irq_is_pending(cfg->uart_dev) > 0) {
        while (uart_irq_rx_ready(cfg->uart_dev) > 0) {
            uint8_t byte;
            uart_fifo_read(cfg->uart_dev, &byte, 1);
            pokprotocol_feed_byte(data, byte);
        }
    }
}

static int pokmac_init(const struct device *dev)
{
    int rc;
    const struct pokmac_cfg *cfg = dev->config;
    struct pokmac_data *data = dev->data;

    LOG_DBG("Initializing %s", dev->name);
    data->self = dev;

    if (!device_is_ready(cfg->uart_dev)) {
        return -ENODEV;
    }

    sys_slist_init(&data->recv_clbks);
    k_mutex_init(&data->uart_mutex);
    k_work_init(&data->send_ack_work, send_ack_work_handler);
    k_event_init(&data->rx_events);

    uart_irq_rx_disable(cfg->uart_dev);
    uart_irq_tx_disable(cfg->uart_dev);

    rc = uart_irq_callback_user_data_set(cfg->uart_dev, pokmac_uart_isr, (void *)dev);
    if (rc != 0) {
        LOG_ERR("UART IRQ setup failed: %d", rc);
        return rc;
    }

    uart_irq_rx_enable(cfg->uart_dev);

    return 0;
}

static const struct pokmac_driver_api pokmac_api = {.register_recv_clbk = register_recv_callback,
                                                    .send = send};

#define POKMAC_INIT(n)                                                                             \
                                                                                                   \
    static struct pokmac_data pokmac_data_##n = {                                                  \
        .receive_index = 0,                                                                        \
        .receiving = false,                                                                        \
    };                                                                                             \
                                                                                                   \
    static const struct pokmac_cfg pokmac_cfg_##n = {                                              \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                                 \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pokmac_init, PM_DEVICE_DT_INST_GET(n), &pokmac_data_##n,              \
                          &pokmac_cfg_##n, POST_KERNEL, CONFIG_POKMAC_INIT_PRIORITY, &pokmac_api);

DT_INST_FOREACH_STATUS_OKAY(POKMAC_INIT)
