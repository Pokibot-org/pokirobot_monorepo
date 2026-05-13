#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/byteorder.h>
#include <pokibot/drivers/pokstepper.h>
#include <sys/errno.h>
#include <stdint.h>
#include <string.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

LOG_MODULE_REGISTER(mks_pokstepper, CONFIG_POKSTEPPER_LOG_LEVEL);

/* MKS ServoCAN command codes */
/* System parameter commands */
#define MKS_CMD_CALIBRATE          0x80
#define MKS_CMD_SET_WORK_MODE      0x82
#define MKS_CMD_SET_CURRENT        0x83
#define MKS_CMD_SET_MSTEP          0x84
#define MKS_CMD_SET_EN_ACTIVE      0x85
#define MKS_CMD_SET_DIRECTION      0x86
#define MKS_CMD_SET_AUTO_SLEEP     0x87
#define MKS_CMD_SET_PROTECT        0x88
#define MKS_CMD_SET_INTERPOLATOR   0x89
#define MKS_CMD_SET_CAN_RATE       0x8A
#define MKS_CMD_SET_CAN_ID         0x8B
#define MKS_CMD_SET_CAN_RESPONSE   0x8C
#define MKS_CMD_SET_GROUP_ID       0x8D
#define MKS_CMD_SET_KEYLOCK        0x8F
#define MKS_CMD_SET_HOME_PARAMS    0x90
#define MKS_CMD_GO_HOME            0x91
#define MKS_CMD_SET_ZERO_POINT     0x92
#define MKS_CMD_SET_NO_LIMIT_RETURN 0x94
#define MKS_CMD_SET_ZERO_MODE      0x9A
#define MKS_CMD_SET_HOLD_CURRENT   0x9B
#define MKS_CMD_SET_LIMIT_REMAP    0x9E
#define MKS_CMD_SET_EN_TRIGGER     0x9D
#define MKS_CMD_READ_SYSTEM_PARAM  0x00
#define MKS_CMD_RESTORE_DEFAULTS   0x3F
#define MKS_CMD_SPEED_STATE        0xFF
#define MKS_CMD_POS_AXIS           0xF4

/* Motion Control Commands */
#define MKS_CMD_SPEED_MODE         0xF6
#define MKS_CMD_EMERGENCY_STOP     0xF7
#define MKS_CMD_QUERY_STATUS       0xF1
#define MKS_CMD_ENABLE_MOTOR       0xF3
#define MKS_CMD_POS_RELATIVE       0xFD
#define MKS_CMD_POS_ABSOLUTE       0xFE

/* Read Commands */
#define MKS_CMD_READ_ENCODER_CARRY 0x30
#define MKS_CMD_READ_ENCODER_ADD   0x31
#define MKS_CMD_READ_SPEED         0x32
#define MKS_CMD_READ_PULSES        0x33
#define MKS_CMD_READ_IO_STATUS     0x34
#define MKS_CMD_READ_RAW_ENCODER   0x35
#define MKS_CMD_READ_ANGLE_ERROR   0x39

#define MKS_CMD_RESTART            0x41


#define MKS_WORK_MODE_CR_FOC       0x02
#define MKS_WORK_MODE_SR_OPEN      0x03
#define MKS_WORK_MODE_SR_CLOSE     0x04
#define MKS_WORK_MODE_SR_FOC       0x05

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


/* System Parameter Commands */
static int mks_pokstepper_calibrate(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_CALIBRATE, 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u calibration started", config->motor_id);
    }
    return ret;
}

static int mks_pokstepper_set_work_mode(const struct device *dev, uint8_t mode)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_WORK_MODE, mode};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u work mode set to 0x%02x", config->motor_id, mode);
    }
    return ret;
}

static int mks_pokstepper_set_current(const struct device *dev, uint16_t current_ma)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_CURRENT, (uint8_t)(current_ma >> 8), (uint8_t)(current_ma & 0xFF)};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u current set to %u mA", config->motor_id, current_ma);
    }
    return ret;
}

static int mks_pokstepper_set_en_active(const struct device *dev, uint8_t en_active)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_EN_ACTIVE, en_active};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_direction(const struct device *dev, uint8_t direction)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_DIRECTION, direction};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u direction set to %u", config->motor_id, direction);
    }
    return ret;
}

static int mks_pokstepper_set_auto_sleep(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_AUTO_SLEEP, enable ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_protect(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_PROTECT, enable ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_interpolator(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_INTERPOLATOR, enable ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_hold_current(const struct device *dev, uint8_t percent)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_HOLD_CURRENT, percent};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_can_rate(const struct device *dev, uint8_t rate)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_CAN_RATE, rate};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_can_id(const struct device *dev, uint16_t new_id)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_CAN_ID, (uint8_t)(new_id >> 8), (uint8_t)(new_id & 0xFF)};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_can_response(const struct device *dev, bool response, bool active)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_CAN_RESPONSE, response ? 0x01 : 0x00, active ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_group_id(const struct device *dev, uint16_t group_id)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_GROUP_ID, (uint8_t)(group_id >> 8), (uint8_t)(group_id & 0xFF)};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_keylock(const struct device *dev, bool lock)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_KEYLOCK, lock ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

/* Home Commands */
static int mks_pokstepper_set_home_params(const struct device *dev, uint8_t type, uint8_t direction,
                                          uint16_t speed, bool enable_limit, uint8_t mode)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_HOME_PARAMS, type, direction, (uint8_t)(speed >> 8),
                         (uint8_t)(speed & 0xFF), enable_limit ? 0x01 : 0x00, mode};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_go_home(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_GO_HOME};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u going home", config->motor_id);
    }
    return ret;
}

static int mks_pokstepper_set_zero_point(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_ZERO_POINT};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        data->current_position = 0;
        LOG_INF("Motor %u zero point set", config->motor_id);
    }
    return ret;
}

static int mks_pokstepper_set_no_limit_return(const struct device *dev, uint32_t return_steps,
                                              uint16_t max_current_ma)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_NO_LIMIT_RETURN,
                         (uint8_t)(return_steps >> 24),
                         (uint8_t)(return_steps >> 16),
                         (uint8_t)(return_steps >> 8),
                         (uint8_t)(return_steps & 0xFF),
                         (uint8_t)(max_current_ma >> 8),
                         (uint8_t)(max_current_ma & 0xFF)};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_limit_remap(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_LIMIT_REMAP, enable ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_zero_mode(const struct device *dev, uint8_t mode, bool enable,
                                        uint8_t speed, uint8_t direction)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SET_ZERO_MODE, mode, enable ? 0x01 : 0x00, speed, direction};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_en_trigger(const struct device *dev, bool en_trigger, bool pulse_polarity,
                                         uint16_t time_ms, uint16_t error)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t flags = (en_trigger ? 0x01 : 0x00) | (pulse_polarity ? 0x02 : 0x00);
    uint8_t payload[] = {MKS_CMD_SET_EN_TRIGGER, flags, (uint8_t)(time_ms >> 8),
                         (uint8_t)(time_ms & 0xFF), (uint8_t)(error >> 8), (uint8_t)(error & 0xFF)};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_set_mstep(const struct device *dev, uint8_t mstep)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    LOG_INF("Set mstep %d", mstep);
    uint8_t payload[] = {MKS_CMD_SET_MSTEP, mstep};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

static int mks_pokstepper_query_status(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_QUERY_STATUS};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u status query sent", config->motor_id);
    }
    return ret;
}

static int mks_pokstepper_emergency_stop(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_EMERGENCY_STOP};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_WRN("Motor %u emergency stop triggered", config->motor_id);
    }
    return ret;
}

static int mks_pokstepper_restart(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_RESTART};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u restart command sent", config->motor_id);
    }
    return ret;
}

/* Configuration Commands */
static int mks_pokstepper_restore_defaults(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_RESTORE_DEFAULTS};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    if (ret == 0) {
        LOG_INF("Motor %u restoring defaults", config->motor_id);
    }
    return ret;
}

static int mks_pokstepper_set_enable(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_ENABLE_MOTOR, enable ? 0x01 : 0x00};
    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));

    return ret;
}

/* Motion Commands */
static int mks_pokstepper_speed_mode(const struct device *dev, int16_t speed, uint8_t acceleration)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t dir = 1;
    if (speed < 0) {
        dir = 0;
        speed = -speed;
    }
    uint8_t speed_high = (dir << 7) | (uint8_t)((speed >> 8) & 0x0F);
    uint8_t speed_low = (uint8_t)(speed & 0xFF);
    uint8_t payload[] = {MKS_CMD_SPEED_MODE, speed_high, speed_low, acceleration};

    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}

static int mks_pokstepper_speed_mode_stop(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SPEED_MODE, 0x00, 0x00, 0x00};
    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}

static int mks_pokstepper_speed_state(const struct device *dev, bool save)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_SPEED_STATE, save ? 0xC8 : 0xCA};
    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}

static int mks_pokstepper_pos_relative(const struct device *dev, uint32_t position, uint16_t speed,
                                       uint8_t acceleration, bool counter_clockwise)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t speed_high = ((counter_clockwise ? 0x80 : 0x00) | ((speed >> 8) & 0x0F));
    uint8_t payload[] = {MKS_CMD_POS_RELATIVE,
                         speed_high,
                         (uint8_t)(speed & 0xFF),
                         acceleration,
                         (uint8_t)(position >> 16),
                         (uint8_t)(position >> 8),
                         (uint8_t)(position & 0xFF)};

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        data->current_position += (counter_clockwise ? -position : position);
        LOG_INF("Relative position move: %u steps at speed %u", position, speed);
    }
    return ret;
}

static int mks_pokstepper_pos_relative_stop(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_POS_RELATIVE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}

static int mks_pokstepper_pos_absolute(const struct device *dev, int32_t absolute_position,
                                       uint16_t speed, uint8_t acceleration)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_POS_ABSOLUTE,
                         (uint8_t)(speed >> 8),
                         (uint8_t)(speed & 0xFF),
                         acceleration,
                         (uint8_t)(absolute_position >> 16),
                         (uint8_t)(absolute_position >> 8),
                         (uint8_t)(absolute_position & 0xFF)};

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        data->current_position = absolute_position;
        LOG_INF("Absolute position move to %d", absolute_position);
    }
    return ret;
}

static int mks_pokstepper_pos_absolute_stop(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_POS_ABSOLUTE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}

static int mks_pokstepper_pos_axis(const struct device *dev, int32_t axis_position, uint16_t speed,
                                   uint8_t acceleration)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_POS_AXIS,
                         (uint8_t)(speed >> 8),
                         (uint8_t)(speed & 0xFF),
                         acceleration,
                         (uint8_t)(axis_position >> 16),
                         (uint8_t)(axis_position >> 8),
                         (uint8_t)(axis_position & 0xFF)};

    int ret = mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
    if (ret == 0) {
        LOG_INF("Axis position move to %d", axis_position);
    }
    return ret;
}

static int mks_pokstepper_pos_axis_stop(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_POS_AXIS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}

static int mks_pokstepper_read_system_param(const struct device *dev, uint8_t param_code)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    uint8_t payload[] = {MKS_CMD_READ_SYSTEM_PARAM, param_code};
    return mks_send_frame(data->can_dev, config->motor_id, payload, sizeof(payload));
}


/* API cmds */

static int mks_pokstepper_api_enable(const struct device *dev, bool enable)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;

    int ret = mks_pokstepper_set_enable(dev, enable);
    if (ret == 0) {
        data->motor_enabled = enable;
        LOG_INF("Motor %u %s", config->motor_id, enable ? "enabled" : "disabled");
    }

    return ret;
}

static int mks_pokstepper_api_set_speed(const struct device *dev, int32_t speed)
{
    if (speed > 1500) {
        speed = 1500;
    } else if (speed < -1500) {
        speed = -1500;
    }
    return mks_pokstepper_speed_mode(dev, speed, 0);
}

static int mks_pokstepper_api_set_pos(const struct device *dev, int32_t pos)
{
    return -ENODATA;
}

static int mks_pokstepper_api_get_pos(const struct device *dev, int32_t *pos)
{
    return -ENODATA;
}

static int mks_pokstepper_api_move_by(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    return -ENODATA;
}

static int mks_pokstepper_api_move_to(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    return -ENODATA;
}

static int mks_pokstepper_api_go_to_stall(const struct device *dev, uint32_t speed_sps, int32_t steps,
                                      uint8_t detection_threshold)
{
    /* Not directly supported by MKS - use relative position move */
    return mks_pokstepper_api_move_by(dev, speed_sps, steps);
}

static int mks_pokstepper_init(const struct device *dev)
{
    struct mks_pokstepper_data *data = dev->data;
    const struct mks_pokstepper_config *config = dev->config;
    int err;

    LOG_INF("Initializing MKS ServoCAN driver for motor ID 0x%x", config->motor_id);

    /* Get CAN device */
    data->can_dev = config->can_dev;
    if (!device_is_ready(data->can_dev)) {
        LOG_ERR("CAN device not ready");
        return -ENODEV;
    }

    data->current_position = 0;
    data->motor_enabled = true;


    (void)can_stop(data->can_dev);
	err = can_set_mode(data->can_dev, CAN_MODE_NORMAL);
    if (err)
    {
        LOG_ERR("Can set mode err %d", err);
        return -ENODEV;
    }
    err = can_set_bitrate(data->can_dev, 500000);
    if (err)
    {
        LOG_ERR("Can set bitrate err %d", err);
        return -ENODEV;
    }
    err = can_start(data->can_dev);
    if (err)
    {
        LOG_ERR("Can start err %d", err);
        return -ENODEV;
    }

    if (mks_pokstepper_speed_mode(dev, 0, 0))
    {
        LOG_ERR("Cannot set speed 0 on stepper");
        return -ENODEV;
    }
    
    if (mks_pokstepper_set_enable(dev, false))
    {
        LOG_ERR("Cannot set enable stepper");
        return -ENODEV;
    }

    // 0 = 256 mstep
    if (mks_pokstepper_set_mstep(dev, 0))
    {
        LOG_ERR("Cannot write mstep");
        return -ENODEV;
    }

    if (mks_pokstepper_set_work_mode(dev, MKS_WORK_MODE_SR_CLOSE))
    {
        LOG_ERR("Cannot write work mode");
        return -ENODEV;
    }

    if (mks_pokstepper_set_enable(dev, data->motor_enabled))
    {
        LOG_ERR("Cannot enable stepper");
        return -ENODEV;
    }

    LOG_INF("MKS ServoCAN driver initialized successfully");

    return 0;
}

static struct pokstepper_driver_api mks_pokstepper_api = {
    .enable = mks_pokstepper_api_enable,
    .set_speed = mks_pokstepper_api_set_speed,
    .set_pos = mks_pokstepper_api_set_pos,
    .get_pos = mks_pokstepper_api_get_pos,
    .move_by = mks_pokstepper_api_move_by,
    .move_to = mks_pokstepper_api_move_to,
    .go_to_stall = mks_pokstepper_api_go_to_stall,
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

#pragma GCC diagnostic pop