#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/msm.h>
#include <pokibot/drivers/lidar.h>

LOG_MODULE_REGISTER(ld19);

#define DT_DRV_COMPAT zephyr_fhl_ld19

#define MSM_TOPIC_ROOT "lidar/"

#define MAX_LIDAR_POINTS 360

struct ld19_config {
};

struct ld19_data {
};

static int ld19_start(const struct device *dev)
{
}

static int ld19_stop(const struct device *dev)
{
}

static int ld19_register_callback(const struct device *dev, struct lidar_callback *clbk)
{
}

static int ld19_init(const struct device *dev)
{
    struct ld19_data *data = dev->data;

    return 0;
}

static const struct lidar_driver_api ld19_api = {
    .register_callback = ld19_register_callback,
    .start = ld19_start,
    .stop = ld19_stop,
};

#define LD19_DEFINE(inst)                                                                      \
    static struct ld19_data ld19_data_##inst = {};                                         \
                                                                                                   \
    static const struct ld19_config ld19_config_##inst = {}; \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, &ld19_init, NULL, &ld19_data_##inst,                       \
                          &ld19_config_##inst, POST_KERNEL, 90, &ld19_api);

DT_INST_FOREACH_STATUS_OKAY(LD19_DEFINE)
