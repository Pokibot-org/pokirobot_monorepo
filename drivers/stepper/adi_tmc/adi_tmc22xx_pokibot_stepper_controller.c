#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/stepper.h>
#include <pokibot/drivers/shared_uart.h>
#include "adi_tmc_uart.h"

LOG_MODULE_REGISTER(tmc22xx_pokibot, CONFIG_STEPPER_LOG_LEVEL);

struct tmc22xx_pokibot_config {
    const struct device *shared_uart_dev;
    uint8_t address;
};

struct tmc22xx_pokibot_data {
    enum stepper_micro_step_resolution resolution;
    uint8_t irun;
    uint8_t ihold;
    uint8_t iholddelay;
};

static int tmc22xx_pokibot_stepper_enable(const struct device *dev, const bool enable)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_move_by(const struct device *dev, const int32_t micro_steps)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_set_microstep_interval(const struct device *dev,
                                                          const uint64_t microstep_interval_ns)
{

    return 0;
}

static int tmc22xx_pokibot_stepper_set_reference_position(const struct device *dev,
                                                          const int32_t value)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_get_actual_position(const struct device *dev, int32_t *value)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_move_to(const struct device *dev, const int32_t value)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_is_moving(const struct device *dev, bool *is_moving)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_run(const struct device *dev,
                                       const enum stepper_direction direction)
{
    return 0;
}

static int tmc22xx_pokibot_stepper_set_event_callback(const struct device *dev,
                                                      stepper_event_callback_t callback,
                                                      void *user_data)
{
    return 0;
}

static int
tmc22xx_pokibot_stepper_set_micro_step_res(const struct device *dev,
                                           enum stepper_micro_step_resolution micro_step_res)
{
    return 0;
}

static int
tmc22xx_pokibot_stepper_get_micro_step_res(const struct device *dev,
                                           enum stepper_micro_step_resolution *micro_step_res)
{
    struct tmc22xx_pokibot_data *data = dev->data;

    *micro_step_res = data->resolution;
    return 0;
}

static int tmc22xx_pokibot_stepper_init(const struct device *dev)
{
    // const struct tmc22xx_pokibot_config *config = dev->config;
    // struct tmc22xx_pokibot_data *data = dev->data;

    return 0;
}

static DEVICE_API(stepper, tmc22xx_pokibot_stepper_api) = {
    .enable = tmc22xx_pokibot_stepper_enable,
    .move_by = tmc22xx_pokibot_stepper_move_by,
    .is_moving = tmc22xx_pokibot_stepper_is_moving,
    .set_reference_position = tmc22xx_pokibot_stepper_set_reference_position,
    .get_actual_position = tmc22xx_pokibot_stepper_get_actual_position,
    .move_to = tmc22xx_pokibot_stepper_move_to,
    .set_microstep_interval = tmc22xx_pokibot_stepper_set_microstep_interval,
    .run = tmc22xx_pokibot_stepper_run,
    .set_event_callback = tmc22xx_pokibot_stepper_set_event_callback,
    .set_micro_step_res = tmc22xx_pokibot_stepper_set_micro_step_res,
    .get_micro_step_res = tmc22xx_pokibot_stepper_get_micro_step_res,
};

#define TMC22XX_POKIBOT_STEPPER_DEFINE(inst)                                                       \
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
    DEVICE_DT_INST_DEFINE(inst, tmc22xx_pokibot_stepper_init, NULL, &tmc22xx_pokibot_data_##inst,  \
                          &tmc22xx_pokibot_config_##inst, POST_KERNEL,                             \
                          CONFIG_STEPPER_INIT_PRIORITY, &tmc22xx_pokibot_stepper_api);

#define DT_DRV_COMPAT adi_tmc2209_pokibot
DT_INST_FOREACH_STATUS_OKAY(TMC22XX_POKIBOT_STEPPER_DEFINE)
#undef DT_DRV_COMPAT
