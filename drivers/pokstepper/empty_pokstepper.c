#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/pokstepper.h>

LOG_MODULE_REGISTER(empty_pokstepper, CONFIG_POKSTEPPER_LOG_LEVEL);

struct empty_pokstepper_config {
};

struct empty_pokstepper_data {
};

static int empty_pokstepper_enable(const struct device *dev, bool enable)
{
    return 0;
}

static int empty_pokstepper_set_speed(const struct device *dev, int32_t speed)
{
    return 0;
}

static int empty_pokstepper_set_pos(const struct device *dev, int32_t pos)
{
    return 0;
}

static int empty_pokstepper_get_pos(const struct device *dev, int32_t *pos)
{
    *pos = 0;
    return 0;
}

static int empty_pokstepper_move_by(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    return 0;
}

static int empty_pokstepper_move_to(const struct device *dev, uint32_t speed_sps, int32_t steps)
{
    return 0;
}

static int empty_pokstepper_go_to_stall(const struct device *dev, uint32_t speed_sps, int32_t steps,
                                        uint8_t detection_threshold)
{
    return 0;
}

static int empty_pokstepper_init(const struct device *dev)
{
    return 0;
}

static struct pokstepper_driver_api empty_pokstepper_api = {
    .enable = empty_pokstepper_enable,
    .set_speed = empty_pokstepper_set_speed,
    .set_pos = empty_pokstepper_set_pos,
    .get_pos = empty_pokstepper_get_pos,
    .move_by = empty_pokstepper_move_by,
    .move_to = empty_pokstepper_move_to,
    .go_to_stall = empty_pokstepper_go_to_stall,
};

#define EMPTY_POKSTEPPER_DEFINE(inst)                                                            \
    static const struct empty_pokstepper_config empty_pokstepper_config_##inst = {};               \
    static struct empty_pokstepper_data empty_pokstepper_data_##inst = {};                         \
    DEVICE_DT_INST_DEFINE(inst, empty_pokstepper_init, NULL, &empty_pokstepper_data_##inst,        \
                          &empty_pokstepper_config_##inst, POST_KERNEL,                            \
                          CONFIG_POKSTEPPER_INIT_PRIORITY, &empty_pokstepper_api);

#define DT_DRV_COMPAT pokibot_empty_pokstepper
DT_INST_FOREACH_STATUS_OKAY(EMPTY_POKSTEPPER_DEFINE)
#undef DT_DRV_COMPAT
