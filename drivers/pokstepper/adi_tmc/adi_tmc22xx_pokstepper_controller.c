#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/pokstepper.h>
#include <pokibot/drivers/shared_uart.h>
#include "adi_tmc_uart.h"

LOG_MODULE_REGISTER(tmc22xx_pokibot, CONFIG_POKSTEPPER_LOG_LEVEL);

struct tmc22xx_pokibot_config {
    const struct device *shared_uart_dev;
    uint8_t address;
};

struct tmc22xx_pokibot_data {
    uint16_t resolution;
    uint8_t irun;
    uint8_t ihold;
    uint8_t iholddelay;
};

static int tmc22xx_pokstepper_enable(const struct device *dev, bool enable)
{
    return 0;
}

static int tmc22xx_pokstepper_set_speed(const struct device *dev, int32_t speed)
{
    return 0;
}

static int tmc22xx_pokstepper_init(const struct device *dev)
{
    return 0;
}


static struct pokstepper_driver_api tmc22xx_pokstepper_api = {
    .enable = tmc22xx_pokstepper_enable,
    .set_speed = tmc22xx_pokstepper_set_speed,
};

#define TMC22XX_POKSTEPPER_DEFINE(inst)                                                       \
    static const struct tmc22xx_pokibot_config tmc22xx_pokibot_config_##inst = {                   \
        .shared_uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                       \
        .address = DT_INST_PROP(inst, address),                                                            \
    };                                                                                             \
    static struct tmc22xx_pokibot_data tmc22xx_pokibot_data_##inst = {                             \
        .resolution = DT_INST_PROP(inst, micro_step_res),                                          \
        .irun = DT_INST_PROP(inst, irun),                                                         \
        .ihold = DT_INST_PROP(inst, ihold),                                                       \
        .iholddelay = DT_INST_PROP(inst, iholddelay),                                            \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, tmc22xx_pokstepper_init, NULL, &tmc22xx_pokibot_data_##inst,  \
                          &tmc22xx_pokibot_config_##inst, POST_KERNEL,                             \
                          CONFIG_POKSTEPPER_INIT_PRIORITY, &tmc22xx_pokstepper_api);

#define DT_DRV_COMPAT adi_tmc2209_pokibot
DT_INST_FOREACH_STATUS_OKAY(TMC22XX_POKSTEPPER_DEFINE)
#undef DT_DRV_COMPAT
