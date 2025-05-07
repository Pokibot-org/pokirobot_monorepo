/*
 * Copyright (c) 2024, CATIE
 * Copyright (c) 2024, Pokibot org
 * SPDX-License-Identifier: Apache-2.0
 *
 * Original code from CATIE:
 * https://github.com/catie-aq/zephyr_ldrobot-ld19/blob/main/drivers/sensor/ldrobot/ld19/ld19.c
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>
#include <zephyr/drivers/uart.h>
#include <pokibot/drivers/lidar.h>
#include <math.h>

LOG_MODULE_REGISTER(ld19, CONFIG_LIDAR_LOG_LEVEL);

#define DT_DRV_COMPAT zephyr_fhl_ld19

#define LD_M_PI                3.14159265358979323846f
#define LD_DEG_TO_RAD(_deg) ((_deg) / 180.0f *LD_M_PI)
#define MAX_LIDAR_POINTS    600

#define LD19_MAX_BUFFER_SIZE      128
#define LD19_PKG_HEADER           0x54
#define LD19_DATA_PKG_INFO        0x2C
#define LD19_HEALTH_PKG_INFO      0xE0
#define LD19_MANUFACT_PKG_INF     0x0F
#define LD19_DATA_LEN             12 * 3 + 11
#define LD19_DATA_HEALTH_LEN      12
#define LD19_DATA_MANUFACTURE_LEN 12

static const uint8_t crc_table[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

enum ld19_state {
    HEADER,
    VER_LEN,
    DATA,
    DATA_HEALTH,
    DATA_MANUFACTURE,
};

struct ld19_buf {
    uint8_t buf[LD19_MAX_BUFFER_SIZE];
    uint8_t index;
};

struct ld19_config {
    const struct device *uart_dev;
};

struct ld19_data {
    struct ld19_buf rx_buf;
    enum ld19_state state;
    struct lidar_point points[MAX_LIDAR_POINTS];
    size_t nb_points;
    sys_slist_t callbacks;
    struct k_work work_call_clbks;
    float last_angle;
};

typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
} LidarPointStructType;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructType point[12];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARMeasureDataType;

static uint8_t ld19_crc8(const uint8_t *data, uint8_t data_len)
{
    uint8_t crc = 0;
    while (data_len--) {
        crc = crc_table[(crc ^ *data) & 0xff];
        data++;
    }
    return crc;
}

static float ld19_angle_normalize(float a)
{
    a = fmodf(a + LD_M_PI, 2 * LD_M_PI);
    if (a < 0) {
        a += 2 * LD_M_PI;
    }
    return a - LD_M_PI;
}

static void ld19_process_measure(struct ld19_data *data, LiDARMeasureDataType *measure)
{
    bool can_be_published = false;
    float start_angle_deg = measure->start_angle * 0.01f;
    float end_angle_deg = measure->end_angle * 0.01f;
    float diff_deg = end_angle_deg - start_angle_deg;
    if (diff_deg < 0) {
        diff_deg += 360.0f;
    }
    const int nb_measure_points = 12;
    float step_deg = diff_deg / nb_measure_points;

    for (int i = 0; i < nb_measure_points; i++) {
        struct lidar_point current_point = {
            .distance = (float)measure->point[i].distance / 1000,
            .angle = ld19_angle_normalize(-LD_DEG_TO_RAD(start_angle_deg + step_deg * i)),
            .intensity = measure->point[i].intensity,
        };
        data->points[data->nb_points] = current_point;
        if (data->nb_points < MAX_LIDAR_POINTS - 1) {
            data->nb_points++;
        } else {
            LOG_ERR("Not enough space in the lidar driver buffer");
        }

        // Has done a full scan
        if (current_point.angle > data->last_angle) {
            can_be_published = true;
        }
        data->last_angle = current_point.angle;
    }

    if (can_be_published) {
        k_work_submit(&data->work_call_clbks);
    }
}

static void ld19_uart_callback_handler(const struct device *dev, void *user_data)
{
    struct ld19_data *data = ((struct device *)user_data)->data;
    int len = 0;
    uint8_t c;

    if ((uart_irq_update(dev) > 0) && (uart_irq_is_pending(dev) < 0)) {
        return;
    }

    while (uart_irq_rx_ready(dev)) {
        len = uart_fifo_read(dev, &c, 1);

        if (len < 0) {
            LOG_ERR("Failed to read data from UART");
            return;
        }

        switch (data->state) {
            case HEADER:
                if (c == LD19_PKG_HEADER) {
                    data->rx_buf.index = 0;
                    data->rx_buf.buf[data->rx_buf.index++] = c;
                    data->state = VER_LEN;
                }
                break;
            case VER_LEN: {
                if (c == LD19_DATA_PKG_INFO) {
                    data->rx_buf.buf[data->rx_buf.index++] = c;
                    data->state = DATA;
                } else if (c == LD19_HEALTH_PKG_INFO) {
                    data->rx_buf.buf[data->rx_buf.index++] = c;
                    data->state = DATA_HEALTH;
                } else if (c == LD19_MANUFACT_PKG_INF) {
                    data->rx_buf.buf[data->rx_buf.index++] = c;
                    data->state = DATA_MANUFACTURE;
                } else {
                    data->state = HEADER;
                }
                break;
            }
            case DATA:
                data->rx_buf.buf[data->rx_buf.index++] = c;
                if (data->rx_buf.index >= LD19_DATA_LEN) {
                    uint8_t crc = ld19_crc8(data->rx_buf.buf, LD19_DATA_LEN - 1);
                    if (crc == data->rx_buf.buf[LD19_DATA_LEN - 1]) {
                        ld19_process_measure(data, (LiDARMeasureDataType *)data->rx_buf.buf);
                    }

                    data->state = HEADER;
                }
                break;
            case DATA_HEALTH:
                data->rx_buf.buf[data->rx_buf.index++] = c;
                if (data->rx_buf.index == LD19_DATA_HEALTH_LEN) {
                    uint8_t crc = ld19_crc8(data->rx_buf.buf, LD19_DATA_HEALTH_LEN - 1);
                    crc = 0; // unused for now
                    data->state = HEADER;
                }
                break;
            case DATA_MANUFACTURE:
                data->rx_buf.buf[data->rx_buf.index++] = c;
                if (data->rx_buf.index == LD19_DATA_MANUFACTURE_LEN) {
                    uint8_t crc = ld19_crc8(data->rx_buf.buf, LD19_DATA_MANUFACTURE_LEN - 1);
                    crc = 0; // unused for now
                    data->state = HEADER;
                }
                break;
            default:
                data->state = HEADER;
                break;
        }
    }
}

void call_callbacks_work_handler(struct k_work *work)
{
    struct ld19_data *data = CONTAINER_OF(work, struct ld19_data, work_call_clbks);

    sys_snode_t *cur;
    SYS_SLIST_FOR_EACH_NODE(&data->callbacks, cur) {
        struct lidar_callback *cb = CONTAINER_OF(cur, struct lidar_callback, _node);
        cb->clbk(data->points, data->nb_points, cb->user_data);
    }
    data->nb_points = 0;
}

static int ld19_start(const struct device *dev)
{
    const struct ld19_config *config = dev->config;
    uart_irq_rx_enable(config->uart_dev);
    LOG_DBG("Lidar rx start");
    return 0;
}

static int ld19_stop(const struct device *dev)
{
    const struct ld19_config *config = dev->config;
    uart_irq_rx_disable(config->uart_dev);
    LOG_DBG("Lidar rx stop");
    return 0;
}

static int ld19_register_callback(const struct device *dev, struct lidar_callback *clbk)
{
    struct ld19_data *data = dev->data;
    sys_slist_append(&data->callbacks, &clbk->_node);
    return 0;
}

static void ld19_uart_flush(const struct device *dev)
{
    uint8_t c;

    while (uart_fifo_read(dev, &c, 1)) {
        continue;
    }
}

static int ld19_init(const struct device *dev)
{
    const struct ld19_config *config = dev->config;
    struct ld19_data *data = dev->data;

    if (!device_is_ready(config->uart_dev)) {
        LOG_ERR("UART device %s is not ready", config->uart_dev->name);
        return -ENODEV;
    }

    sys_slist_init(&data->callbacks);
    k_work_init(&data->work_call_clbks, call_callbacks_work_handler);
    ld19_uart_flush(config->uart_dev);
    uart_irq_callback_user_data_set(config->uart_dev, ld19_uart_callback_handler, (void *)dev);

    return 0;
}

static const struct lidar_driver_api ld19_api = {
    .register_callback = ld19_register_callback,
    .start = ld19_start,
    .stop = ld19_stop,
};

#define LD19_DEFINE(inst)                                                                          \
    static struct ld19_data ld19_data_##inst = {};                                                 \
                                                                                                   \
    static struct ld19_config ld19_config_##inst = {                                               \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                              \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, &ld19_init, NULL, &ld19_data_##inst, &ld19_config_##inst,          \
                          POST_KERNEL, 90, &ld19_api);

DT_INST_FOREACH_STATUS_OKAY(LD19_DEFINE)
