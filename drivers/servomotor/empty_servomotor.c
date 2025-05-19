#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/servomotor.h>

LOG_MODULE_REGISTER(empty_servomotor, CONFIG_SERVOMOTOR_LOG_LEVEL);

struct empty_servomotor_config {
};

struct empty_servomotor_data {
};

int empty_servomotor_api_set_angle(const struct device *dev, float angle)
{
    return 0;
}

static int empty_servomotor_init(const struct device *dev)
{
    return 0;
}

static struct servomotor_driver_api empty_servomotor_api = {
    .set_angle = empty_servomotor_api_set_angle,
};

#define EMPTY_SERVOMOTOR_DEFINE(inst)                                                            \
    static const struct empty_servomotor_config empty_servomotor_config_##inst = {};               \
    static struct empty_servomotor_data empty_servomotor_data_##inst = {};                         \
    DEVICE_DT_INST_DEFINE(inst, empty_servomotor_init, NULL, &empty_servomotor_data_##inst,        \
                          &empty_servomotor_config_##inst, POST_KERNEL,                            \
                          CONFIG_POKSTEPPER_INIT_PRIORITY, &empty_servomotor_api);

#define DT_DRV_COMPAT pokibot_empty_servomotor
DT_INST_FOREACH_STATUS_OKAY(EMPTY_SERVOMOTOR_DEFINE)
#undef DT_DRV_COMPAT
