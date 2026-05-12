#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <pokibot/drivers/pokstepper.h>

LOG_MODULE_REGISTER(mks_pokstepper, CONFIG_POKSTEPPER_LOG_LEVEL);

/* MKS ServoCAN command codes */
#define MKS_CMD_READ_ENCODER_CARRY 0x30
#define MKS_CMD_READ_ENCODER_ADD   0x31
#define MKS_CMD_READ_SPEED         0x32
#define MKS_CMD_READ_PULSES        0x33
#define MKS_CMD_READ_IO_STATUS     0x34
#define MKS_CMD_READ_RAW_ENCODER   0x35
#define MKS_CMD_READ_ANGLE_ERROR   0x39
#define MKS_CMD_QUERY_STATUS       0xF1
#define MKS_CMD_ENABLE_MOTOR       0xF3
#define MKS_CMD_SPEED_MODE         0xF6
#define MKS_CMD_EMERGENCY_STOP     0xF7
#define MKS_CMD_POS_RELATIVE       0xFD
#define MKS_CMD_POS_ABSOLUTE       0xFE
#define MKS_CMD_CALIBRATE          0x80
#define MKS_CMD_SET_CURRENT        0x83
#define MKS_CMD_SET_WORK_MODE      0x82
#define MKS_CMD_RESTART            0x41

struct mks_pokstepper_config {
    const struct device *can_dev;
    uint32_t motor_id;
};

struct mks_pokstepper_data {
    const struct device *can_dev;
    int32_t current_position;
    bool motor_enabled;
};

/* CRC computation - MKS protocol uses simple 8-bit checksum */
static uint8_t mks_compute_crc(uint32_t id, const uint8_t *data, size_t n)
{
    uint32_t sum = id;
    for (size_t i = 0; i < n; ++i) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* Send a CAN frame with MKS protocol */
static int mks_send_frame(const struct device *can_dev, uint32_t motor_id, const uint8_t *payload,
                          size_t payload_len)
{
    struct can_frame frame = {0};
    uint8_t frame_data[8] = {0};

    if (payload_len > 7) {
        LOG_ERR("Payload too large: %zu", payload_len);
        return -EINVAL;
    }

    /* Copy payload */
    memcpy(frame_data, payload, payload_len);

    /* Compute and append CRC */
    frame_data[payload_len] = mks_compute_crc(motor_id, payload, payload_len);

    frame.id = motor_id;
    frame.dlc = payload_len + 1;
    frame.flags = 0; /* Standard CAN frame */
    memcpy(frame.data, frame_data, frame.dlc);

    int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {
        LOG_ERR("Failed to send CAN frame: %d", ret);
    }
    return ret;
}

static int mks_pokstepper_enable(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_ENABLE_MOTOR, enable ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        data->motor_enabled = enable;
        LOG_INF("Motor %u %s", config->motor_id, enable ? "enabled" : "disabled");
    }

    return ret;
}

static int mks_pokstepper_set_speed(const struct device *dev, int32_t speed)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    /* Speed mode command: 0xF6, speed high bits with direction flag, speed low bits, acceleration
     */
    uint8_t payload[] = {
        MKS_CMD_SPEED_MODE, (uint8_t)((speed >> 8) & 0x0F), (uint8_t)(speed & 0xFF),
        0x00 /* acceleration */
    };

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        LOG_INF("Speed set to %d", speed);
    }
    return ret;
}

static int mks_pokstepper_set_pos(const struct device *dev, int32_t pos)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    /* Absolute position command: 0xFE, speed high bits, speed low bits, acceleration, pos[2:0] */
    uint8_t payload[] = {MKS_CMD_POS_ABSOLUTE,
                         0x02, /* default speed high bits */
                         0x00, /* default speed low bits */
                         0x00, /* default acceleration */
                         (uint8_t)(pos >> 16),
                         (uint8_t)(pos >> 8),
                         (uint8_t)(pos & 0xFF)};

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        data->current_position = pos;
        LOG_INF("Position set to %d", pos);
    }
    return ret;
}

static int mks_pokstepper_get_pos(const struct device *dev, int32_t *pos)
{
    struct mks_pokstepper_data *data = dev->data;

    if (!pos) {
        return -EINVAL;
    }

    *pos = data->current_position;
    return 0;
}

static int mks_pokstepper_move_by(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    /* Relative position command: 0xFD */
    uint8_t payload[] = {MKS_CMD_POS_RELATIVE,        (uint8_t)((speed_sps >> 8) & 0x0F),
                         (uint8_t)(speed_sps & 0xFF), 0x00, /* acceleration */
                         (uint8_t)(steps >> 16),      (uint8_t)(steps >> 8),
                         (uint8_t)(steps & 0xFF)};

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        data->current_position += steps;
        LOG_INF("Move by %d steps at speed %u", steps, speed_sps);
    }
    return ret;
}

static int mks_pokstepper_move_to(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    /* For this driver, move_to is the same as set_pos */
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_POS_ABSOLUTE,        (uint8_t)((speed_sps >> 8) & 0x0F),
                         (uint8_t)(speed_sps & 0xFF), 0x00,
                         (uint8_t)(steps >> 16),      (uint8_t)(steps >> 8),
                         (uint8_t)(steps & 0xFF)};

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        data->current_position = steps;
        LOG_INF("Move to position %d at speed %u", steps, speed_sps);
    }
    return ret;
}

static int mks_pokstepper_go_to_stall(const struct device *dev, uint32_t speed_sps, int32_t steps,
                                      uint8_t detection_threshold)
{
    /* Not directly supported by MKS - use relative position move */
    return mks_pokstepper_move_by(dev, speed_sps, steps);
}

static int mks_pokstepper_init(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    LOG_INF("Initializing MKS ServoCAN driver for motor ID 0x%x", config->motor_id);

    /* Get CAN device */
    data->can_dev = config->can_dev;
    if (!device_is_ready(data->can_dev)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    data->current_position = 0;
    data->motor_enabled = false;

    LOG_INF("MKS ServoCAN driver initialized successfully");

    return 0;
}

static struct pokstepper_driver_api mks_pokstepper_api = {
    .enable = mks_pokstepper_enable,
    .set_speed = mks_pokstepper_set_speed,
    .set_pos = mks_pokstepper_set_pos,
    .get_pos = mks_pokstepper_get_pos,
    .move_by = mks_pokstepper_move_by,
    .move_to = mks_pokstepper_move_to,
    .go_to_stall = mks_pokstepper_go_to_stall,
};

#define MKS_POKSTEPPER_DEFINE(inst)                                                                \
    static const struct mks_pokstepper_config mks_pokstepper_config_##inst = {                     \
        .can_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, can_device)),                               \
        .motor_id = DT_INST_PROP(inst, motor_id),                                                  \
    };                                                                                             \
    static struct mks_pokstepper_data mks_pokstepper_data_##inst = {};                             \
    DEVICE_DT_INST_DEFINE(inst, mks_pokstepper_init, NULL, &mks_pokstepper_data_##inst,            \
                          &mks_pokstepper_config_##inst, POST_KERNEL,                              \
                          CONFIG_POKSTEPPER_INIT_PRIORITY, &mks_pokstepper_api);

#define DT_DRV_COMPAT pokibot_mks_pokstepper
DT_INST_FOREACH_STATUS_OKAY(MKS_POKSTEPPER_DEFINE)
#undef DT_DRV_COMPAT
