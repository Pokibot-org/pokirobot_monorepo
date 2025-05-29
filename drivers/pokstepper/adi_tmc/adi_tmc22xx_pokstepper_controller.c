#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <pokibot/drivers/pokstepper.h>
#include <pokibot/drivers/uart_bus.h>
#include <stdlib.h>
#include "adi_tmc_uart.h"

LOG_MODULE_REGISTER(tmc22xx_pokibot, CONFIG_POKSTEPPER_LOG_LEVEL);

#define RESOLUTION_TO_MRES(x)      (LOG2(256) - LOG2(x))
// #define TMC2209_HZ_TO_CLOCK_FREQ(x) ((x) * 0.715f)
// values
#define TMC2209_VACTUAL_MAX        ((1 << 23) - 1)
#define TMC2209_VACTUAL_MIN        (-TMC2209_VACTUAL_MAX)
#define TMC2209_GCONF_DEFAULT      0x00000101
#define TMC2209_CHOPCONF_DEFAULT   0x10000053
#define TMC2209_IHOLD_IRUN_DEFAULT 0x000F1F00

// registers
#define TMC2209_REG_GCONF        0x00
#define TMC2209_REG_GSTAT        0x01
#define TMC2209_REG_IFCNT        0x02
#define TMC2209_REG_SLAVECONF    0x03
#define TMC2209_REG_OTP_PROG     0x04
#define TMC2209_REG_OTP_READ     0x05
#define TMC2209_REG_IOIN         0x06
#define TMC2209_REG_FACTORY_CONF 0x07

#define TMC2209_REG_IHOLD_IRUN  0x10
#define TMC2209_REG_TPOWER_DOWN 0x11
#define TMC2209_REG_TSTEP       0x12
#define TMC2209_REG_TPWMTHRS    0x13
#define TMC2209_REG_TCOOLTHRS   0x14

#define TMC2209_REG_VACTUAL 0x22

#define TMC2209_REG_SGTHRS    0x40
#define TMC2209_REG_SG_RESULT 0x41
#define TMC2209_REG_COOLCONF  0x42

#define TMC2209_REG_MSCNT      0x6A
#define TMC2209_REG_MSCURACT   0x6B
#define TMC2209_REG_CHOPCONF   0x6C
#define TMC2209_REG_DRV_STATUS 0x6F
#define TMC2209_REG_PWMCONF    0x70
#define TMC2209_REG_PWM_SCALE  0x71
#define TMC2209_REG_PWM_AUTO   0x72

// error and warning codes
#define TMC2209_ERR_RREPLY_CRC  0xFF01
#define TMC2209_ERR_SPEED_RANGE 0x2201

#define MOVE_EVENT_OK    BIT(1)
#define MOVE_EVENT_STALL BIT(2)

#define STEPPER_OK    0
#define STEPPER_STALL 1

struct tmc22xx_pokibot_config {
    const struct device *uart_bus_dev;
    struct gpio_dt_spec en;
    struct gpio_dt_spec nstdby;
    struct gpio_dt_spec step_pin;
    struct gpio_dt_spec dir_pin;
    const struct device *counter;
    uint8_t address;
    int sg_nb_consecutive_trigger;
    int sg_poll_period_ms;
    int initial_sg_poll_period_ms;
};

struct tmc22xx_pokibot_data {
    const struct device *self;

    int32_t pos;
    int32_t mstep_pos_in_fullstep;
    bool has_known_pos;
    uint16_t resolution;
    uint8_t irun;
    uint8_t ihold;
    uint8_t iholddelay;
    bool reverse_shaft;

    // STEP DIR CONTROL
    int32_t target_pos;
    int32_t pos_increment;
    struct k_event move_events;

    uint8_t sg_threshold;
    uint8_t nb_sg_confirmations;
    struct k_work_delayable poll_stallguard_dwork;

    struct counter_top_cfg counter_top_cfg;
    // SHADOW REGS
    uint32_t chopconf;
    uint32_t gconf;
};

static int tmc22xx_wrequest(const struct device *dev, uint8_t reg, uint32_t data)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    return tmc_reg_write(config->uart_bus_dev, config->address, reg, data);
}

static int tmc22xx_rrequest(const struct device *dev, uint8_t reg, uint32_t *data)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    return tmc_reg_read(config->uart_bus_dev, config->address, reg, data);
}

static int tmc22xx_get_ifcnt(const struct device *dev, uint32_t *ifcnt)
{
    return tmc22xx_rrequest(dev, TMC2209_REG_IFCNT, ifcnt);
}

static int tmc22xx_wrequest_confirmed(const struct device *dev, uint8_t reg, uint32_t data)
{
    const int nb_retry = 5;
    uint32_t ifcnt_before, ifcnt_after;
    for (int i = 0; i < nb_retry; i++) {
        tmc22xx_get_ifcnt(dev, &ifcnt_before);
        tmc22xx_wrequest(dev, reg, data);
        tmc22xx_get_ifcnt(dev, &ifcnt_after);
        if ((ifcnt_before + 1) == ifcnt_after) {
            return 0;
        }
    }
    LOG_ERR("Write confirmed error");
    return -1;
}

static int tmc22xx_set_senddelay(const struct device *dev, uint32_t senddelay)
{
    uint32_t data = FIELD_PREP(GENMASK(11, 8), senddelay);
    return tmc22xx_wrequest_confirmed(dev, TMC2209_REG_SLAVECONF, data);
}

static int tmc22xx_set_ihold_irun(const struct device *dev, uint32_t ihold, uint32_t irun,
                                  uint32_t iholddelay)
{
    int ret = 0;
    uint32_t data = FIELD_PREP(GENMASK(4, 0), ihold) | FIELD_PREP(GENMASK(12, 8), irun) |
                    FIELD_PREP(GENMASK(19, 16), iholddelay);
    ret = tmc22xx_wrequest_confirmed(dev, TMC2209_REG_IHOLD_IRUN, data);
    return ret;
}

// static int tmc22xx_set_freewheel_conf(const struct device *dev, uint8_t freewheel)
// {
//     uint32_t reg;
//     if (tmc22xx_rrequest(dev, TMC2209_REG_PWMCONF, &reg)) {
//         return -1;
//     }
//     reg = (reg & ~GENMASK(21, 20)) | FIELD_PREP(GENMASK(21, 20), freewheel);
//     return tmc22xx_wrequest(dev, TMC2209_REG_PWMCONF, reg);
// }

static int tmc22xx_set_mres(const struct device *dev, uint32_t mres)
{
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    int ret = 0;
    data->gconf = (data->gconf & ~(GENMASK(7, 7) | GENMASK(3, 3))) | FIELD_PREP(GENMASK(7, 7), 1) |
                  FIELD_PREP(GENMASK(3, 3), data->reverse_shaft ? 1 : 0);
    ret |= tmc22xx_wrequest_confirmed(dev, TMC2209_REG_GCONF, data->gconf);
    data->chopconf = (data->chopconf & ~GENMASK(27, 24)) | FIELD_PREP(GENMASK(27, 24), mres);
    ret |= tmc22xx_wrequest_confirmed(dev, TMC2209_REG_CHOPCONF, data->chopconf);
    return ret;
}

static int tmc22xx_enable_uart(const struct device *dev, bool enable)
{
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    data->chopconf =
        (data->chopconf & ~GENMASK(3, 0)) | FIELD_PREP(GENMASK(3, 0), enable ? 0xF : 0);
    return tmc22xx_wrequest_confirmed(dev, TMC2209_REG_CHOPCONF, data->chopconf);
}

static int tmc22xx_get_sg_result(const struct device *dev, uint8_t *sg_result)
{
    uint32_t data = 510;
    int ret = tmc22xx_rrequest(dev, TMC2209_REG_SG_RESULT, &data);
    *sg_result = data >> 1;
    return ret;
}

static int tmc22xx_pokstepper_enable(const struct device *dev, bool enable)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;

    if (gpio_is_ready_dt(&config->en)) {
        gpio_pin_set_dt(&config->en, enable);
    } else {
        tmc22xx_enable_uart(dev, enable);
    }
    return 0;
}

static int tmc22xx_pokstepper_set_speed(const struct device *dev, int32_t speed)
{
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    if (speed < TMC2209_VACTUAL_MIN || speed > TMC2209_VACTUAL_MAX) {
        return TMC2209_ERR_SPEED_RANGE;
    }
    tmc22xx_wrequest(dev, TMC2209_REG_VACTUAL, speed);
    data->has_known_pos = false;
    return 0;
}

static bool tmc22xx_can_do_step_dir(const struct device *dev)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    if (!device_is_ready(config->counter)) {
        return false;
    }
    if (!gpio_is_ready_dt(&config->step_pin)) {
        return false;
    }
    if (!gpio_is_ready_dt(&config->dir_pin)) {
        return false;
    }
    return true;
}

static int tmc22xx_pokstepper_set_pos(const struct device *dev, int32_t pos)
{
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    data->pos = pos;
    return 0;
}

static int tmc22xx_pokstepper_get_pos(const struct device *dev, int32_t *pos)
{
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    *pos = data->pos;
    return 0;
}

static void step_counter_top_interrupt(const struct device *_dev, void *user_data)
{
    ARG_UNUSED(_dev);
    const struct device *stepper_dev = user_data;
    const struct tmc22xx_pokibot_config *config =
        (struct tmc22xx_pokibot_config *)stepper_dev->config;
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)stepper_dev->data;

    gpio_pin_toggle_dt(&config->step_pin);
    if (!gpio_pin_get_dt(&config->step_pin)) {
        data->mstep_pos_in_fullstep += 1;
        if (data->mstep_pos_in_fullstep != data->resolution) {
            return;
        }
        data->mstep_pos_in_fullstep = 0;
        data->pos += data->pos_increment;
        if (data->pos == data->target_pos) {
            counter_stop(config->counter);
            k_event_set(&data->move_events, MOVE_EVENT_OK);
        }
    }
}

static void poll_stallguard_dwork_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tmc22xx_pokibot_data *data =
        CONTAINER_OF(dwork, struct tmc22xx_pokibot_data, poll_stallguard_dwork);
    const struct device *dev = data->self;
    const struct tmc22xx_pokibot_config *config = dev->config;

    uint8_t sg;
    tmc22xx_get_sg_result(dev, &sg);
    // LOG_DBG("sg %d", sg);
    if (sg <= data->sg_threshold) {
        data->nb_sg_confirmations += 1;
    } else {
        data->nb_sg_confirmations = 0;
    }

    if (data->nb_sg_confirmations >= config->sg_nb_consecutive_trigger) {
        counter_stop(config->counter);
        k_event_set(&data->move_events, MOVE_EVENT_STALL);
    } else {
        k_work_reschedule(dwork, K_MSEC(config->sg_poll_period_ms));
    }

#if CONFIG_POKSTEPPER_LOG_LEVEL_DBG
    static uint32_t last_print_ms = 0;
    uint8_t smallest_sg = 0;
    if (sg < smallest_sg) {
        smallest_sg = sg;
    }
    if (k_uptime_get_32() - last_print_ms > 1000) {
        LOG_DBG("sg %d", sg);
        last_print_ms = k_uptime_get_32();
        smallest_sg = 0;
    }
#endif
}

static int move(const struct device *dev, uint32_t speed_msps, int32_t target_pos,
                uint8_t stall_threshold)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    if (!tmc22xx_can_do_step_dir(dev)) {
        return -ENODEV;
    }

    data->target_pos = target_pos;
    int32_t steps = target_pos - data->pos;
    if (!steps) {
        return STEPPER_OK;
    }

    bool direction = steps >= 0;
    data->pos_increment = direction ? +1 : -1;
    data->mstep_pos_in_fullstep = 0;
    gpio_pin_set_dt(&config->dir_pin, direction);
    uint32_t step_interval_ns = NSEC_PER_SEC / speed_msps;
    LOG_DBG("step_interval_ns %d", step_interval_ns);

    data->counter_top_cfg.ticks = DIV_ROUND_UP(
        (uint64_t)counter_get_frequency(config->counter) * step_interval_ns / 2, NSEC_PER_SEC);
    LOG_DBG("Top counter ticks %d", data->counter_top_cfg.ticks);
    if (counter_set_top_value(config->counter, &data->counter_top_cfg)) {
        LOG_ERR("Counter cant set top value");
        return -EIO;
    }

    k_event_clear(&data->move_events, 0xFFFFFFFF);
    uint32_t events_to_wait = MOVE_EVENT_OK;
    if (stall_threshold) {
        data->sg_threshold = stall_threshold;
        data->nb_sg_confirmations = 0;
        events_to_wait |= MOVE_EVENT_STALL;
        k_work_reschedule(&data->poll_stallguard_dwork, K_MSEC(config->initial_sg_poll_period_ms));
    }

    int err = counter_start(config->counter);
    if (err) {
        LOG_ERR("Counter start error %d", err);
        return err;
    }
    uint32_t events = k_event_wait(&data->move_events, events_to_wait, false,
                                   K_MSEC(100 + 1000 * (abs(steps * data->resolution)) / speed_msps));
    counter_stop(config->counter);
    if (events & MOVE_EVENT_OK) {
        LOG_DBG("Move ok");
        return STEPPER_OK;
    } else if (events & MOVE_EVENT_STALL) {
        LOG_DBG("Move stall");
        return STEPPER_STALL;
    }
    return -ETIMEDOUT;
}

static int tmc22xx_pokstepper_move_by(const struct device *dev, uint32_t speed_msps, int32_t steps)
{
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    return move(dev, speed_msps, data->pos + steps, 0);
}

static int tmc22xx_pokstepper_move_to(const struct device *dev, uint32_t speed_msps, int32_t steps)
{
    return move(dev, speed_msps, steps, 0);
}

static int tmc22xx_pokstepper_go_to_stall(const struct device *dev, uint32_t speed_msps,
                                          int32_t steps, uint8_t detection_threshold)
{
    int ret = move(dev, speed_msps, steps, detection_threshold);
    if (ret == STEPPER_STALL) {
        return 0;
    } else if (ret == STEPPER_OK) {
        return -1;
    }
    return ret;
}

static int tmc22xx_pokstepper_init(const struct device *dev)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;
    int ret = 0;

    data->self = dev;

    if (gpio_is_ready_dt(&config->en)) {
        gpio_pin_set_dt(&config->en, 1);
    }

    if (gpio_is_ready_dt(&config->nstdby)) {
        gpio_pin_set_dt(&config->nstdby, 1);
    }

    ret |= tmc22xx_set_senddelay(dev, 2);
    ret |= tmc22xx_set_mres(dev, RESOLUTION_TO_MRES(data->resolution));
    ret |= tmc22xx_set_ihold_irun(dev, data->ihold, data->irun, data->iholddelay);
    ret |= tmc22xx_pokstepper_set_speed(dev, 0);
    if (ret) {
        LOG_ERR("tmc conf incorrect");
    }

    if (tmc22xx_can_do_step_dir(dev)) {
        LOG_DBG("Can do step dir");
        LOG_DBG("Counter HZ(%d), max value (%d)", counter_get_frequency(config->counter),
                counter_get_max_top_value(config->counter));

        data->counter_top_cfg.callback = step_counter_top_interrupt;
        data->counter_top_cfg.user_data = (void *)dev;
        data->counter_top_cfg.flags = 0;
        data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);

        ret = gpio_pin_configure_dt(&config->step_pin, GPIO_OUTPUT_LOW);
        if (ret < 0) {
            LOG_ERR("Failed to configure step pin: %d", ret);
            return ret;
        }

        ret = gpio_pin_configure_dt(&config->dir_pin, GPIO_OUTPUT_LOW);
        if (ret < 0) {
            LOG_ERR("Failed to configure dir pin: %d", ret);
            return ret;
        }

        k_event_init(&data->move_events);
        k_work_init_delayable(&data->poll_stallguard_dwork, poll_stallguard_dwork_handler);
    }
    LOG_DBG("tmc22xx <%p> init ok", (void *)dev);
    return 0;
}

static struct pokstepper_driver_api tmc22xx_pokstepper_api = {
    .enable = tmc22xx_pokstepper_enable,
    .set_speed = tmc22xx_pokstepper_set_speed,
    .set_pos = tmc22xx_pokstepper_set_pos,
    .get_pos = tmc22xx_pokstepper_get_pos,
    .move_by = tmc22xx_pokstepper_move_by,
    .move_to = tmc22xx_pokstepper_move_to,
    .go_to_stall = tmc22xx_pokstepper_go_to_stall,
};

#define TMC22XX_POKSTEPPER_DEFINE(inst)                                                            \
    static const struct tmc22xx_pokibot_config tmc22xx_pokibot_config_##inst = {                   \
        .nstdby = GPIO_DT_SPEC_INST_GET_OR(DT_DRV_INST(inst), nstdby_gpios, {0}),                  \
        .en = GPIO_DT_SPEC_INST_GET_OR(DT_DRV_INST(inst), en_gpios, {0}),                          \
        .uart_bus_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                          \
        .address = DT_INST_PROP(inst, address),                                                    \
        .sg_nb_consecutive_trigger = DT_INST_PROP_OR(inst, sg_nb_consecutive_trigger, 2),          \
        .sg_poll_period_ms = DT_INST_PROP_OR(inst, sg_poll_period_ms, 2),                          \
        .initial_sg_poll_period_ms = DT_INST_PROP_OR(inst, initial_sg_poll_period_ms, 1),          \
        .step_pin = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), step_gpios, {0}),                       \
        .dir_pin = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(inst), dir_gpios, {0}),                         \
        .counter = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(DT_DRV_INST(inst), counter)),                  \
    };                                                                                             \
    static struct tmc22xx_pokibot_data tmc22xx_pokibot_data_##inst = {                             \
        .resolution = DT_INST_PROP(inst, micro_step_res),                                          \
        .irun = DT_INST_PROP(inst, irun),                                                          \
        .ihold = DT_INST_PROP(inst, ihold),                                                        \
        .iholddelay = DT_INST_PROP(inst, iholddelay),                                              \
        .reverse_shaft = DT_INST_PROP_OR(inst, reverse_shaft, false),                              \
        .gconf = TMC2209_GCONF_DEFAULT,                                                            \
        .chopconf = TMC2209_CHOPCONF_DEFAULT,                                                      \
        .pos = 0,                                                                                  \
        .has_known_pos = false};                                                                   \
    DEVICE_DT_INST_DEFINE(inst, tmc22xx_pokstepper_init, NULL, &tmc22xx_pokibot_data_##inst,       \
                          &tmc22xx_pokibot_config_##inst, POST_KERNEL,                             \
                          CONFIG_POKSTEPPER_INIT_PRIORITY, &tmc22xx_pokstepper_api);

#define DT_DRV_COMPAT adi_tmc2209_pokibot
DT_INST_FOREACH_STATUS_OKAY(TMC22XX_POKSTEPPER_DEFINE)
#undef DT_DRV_COMPAT
