#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <pokibot/drivers/pokstepper.h>
#include <pokibot/drivers/shared_uart.h>
#include "adi_tmc_uart.h"

LOG_MODULE_REGISTER(tmc22xx_pokibot, CONFIG_POKSTEPPER_LOG_LEVEL);

#define RESOLUTION_TO_MRES(x)      (LOG2(256) - LOG2(x))
// values
#define TMC2209_VACTUAL_MAX        ((1 << 23) - 1)
#define TMC2209_VACTUAL_MIN        (-TMC2209_VACTUAL_MAX)
#define TMC2209_GCONF_DEFAULT      0x00000101
#define TMC2209_CHOPCONF_DEFAULT   0x10000053
#define TMC2209_IHOLD_IRUN_DEFAULT 0x000F1F00
#define TMC2209_MSTEP_ADDR_SELECT  1

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

struct tmc22xx_pokibot_config {
    const struct device *shared_uart_dev;
    struct gpio_dt_spec en;
    struct gpio_dt_spec nstdby;
    uint8_t address;
};

struct tmc22xx_pokibot_data {
    uint16_t resolution;
    uint8_t irun;
    uint8_t ihold;
    uint8_t iholddelay;
};

static int tmc22xx_wrequest(const struct device *dev, uint8_t reg, uint32_t data)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    return tmc_reg_write(config->shared_uart_dev, config->address, reg, data);
}

static int tmc22xx_rrequest(const struct device *dev, uint8_t reg, uint32_t *data)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    return tmc_reg_read(config->shared_uart_dev, config->address, reg, data);
}

static int tmc22xx_set_senddelay(const struct device *dev, uint32_t senddelay)
{
    int ret = 0;
    uint32_t data = FIELD_PREP(GENMASK(11, 8), senddelay);
    ret = tmc22xx_wrequest(dev, TMC2209_REG_SLAVECONF, data);
    return ret;
}

static int tmc22xx_set_ihold_irun(const struct device *dev, uint32_t ihold, uint32_t irun,
                                  uint32_t iholddelay)
{
    int ret = 0;
    uint32_t data = FIELD_PREP(GENMASK(4, 0), ihold) | FIELD_PREP(GENMASK(12, 8), irun) |
                    FIELD_PREP(GENMASK(19, 16), iholddelay);
    ret = tmc22xx_wrequest(dev, TMC2209_REG_IHOLD_IRUN, data);
    return ret;
}

static int tmc22xx_set_mres(const struct device *dev, uint32_t mres)
{
    int ret = 0;
    uint32_t gconf =
        (TMC2209_GCONF_DEFAULT & !FIELD_PREP(GENMASK(7, 7), 0)) | FIELD_PREP(GENMASK(7, 7), 1);
    ret |= tmc22xx_wrequest(dev, TMC2209_REG_GCONF, gconf);
    uint32_t chopconf = (TMC2209_CHOPCONF_DEFAULT & !FIELD_PREP(GENMASK(27, 24), 0)) |
                        FIELD_PREP(GENMASK(27, 24), mres);
    ret |= tmc22xx_wrequest(dev, TMC2209_REG_CHOPCONF, chopconf);
    return ret;
}

static int tmc22xx_get_ifcnt(const struct device *dev, uint32_t *ifcnt)
{
    int ret = 0;
    ret = tmc22xx_rrequest(dev, TMC2209_REG_IFCNT, ifcnt);
    return ret;
}

// static int tmc22xx_get_gconf(const struct device *dev, uint32_t *gconf)
// {
//     int ret = 0;
//     ret = tmc22xx_rrequest(dev, TMC2209_REG_GCONF, gconf);
//     return ret;
// }

// static int tmc22xx_get_stallguard(const struct device *dev, uint32_t *sg)
// {
//     int ret = 0;
//     ret = tmc22xx_rrequest(dev, TMC2209_REG_SG_RESULT, sg);
//     return ret;
// }

static int tmc22xx_pokstepper_enable(const struct device *dev, bool enable)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    if (gpio_is_ready_dt(&config->en)) {
        gpio_pin_set_dt(&config->en, enable);
    } else {
        if (!enable) {
            LOG_ERR("Not implemented, driver always on");
            return -1;
        }
    }
    return 0;
}

static int tmc22xx_pokstepper_set_speed(const struct device *dev, int32_t speed)
{
    if (speed < TMC2209_VACTUAL_MIN || speed > TMC2209_VACTUAL_MAX) {
        return TMC2209_ERR_SPEED_RANGE;
    }
    tmc22xx_wrequest(dev, TMC2209_REG_VACTUAL, speed);
    return 0;
}

static int tmc22xx_pokstepper_init(const struct device *dev)
{
    const struct tmc22xx_pokibot_config *config = (struct tmc22xx_pokibot_config *)dev->config;
    struct tmc22xx_pokibot_data *data = (struct tmc22xx_pokibot_data *)dev->data;

    if (gpio_is_ready_dt(&config->en)) {
        gpio_pin_set_dt(&config->en, 1);
    }

    if (gpio_is_ready_dt(&config->nstdby)) {
        gpio_pin_set_dt(&config->nstdby, 1);
    }

    uint32_t ifcnt;
    tmc22xx_get_ifcnt(dev, &ifcnt);
    tmc22xx_set_senddelay(dev, 2);
    tmc22xx_set_mres(dev, RESOLUTION_TO_MRES(data->resolution));
    tmc22xx_set_ihold_irun(dev, data->ihold, data->irun, data->iholddelay);
    tmc22xx_pokstepper_set_speed(dev, 0);
    uint32_t ifcnt_after;
    tmc22xx_get_ifcnt(dev, &ifcnt_after);
    if (ifcnt_after - ifcnt != 5) {
        LOG_ERR("tmc conf incorrect ifcnt %d | ifcnt_after %d", ifcnt, ifcnt_after);
    }
    LOG_DBG("tmc22xx <%p> init ok", (void *)dev);
    return 0;
}

static struct pokstepper_driver_api tmc22xx_pokstepper_api = {
    .enable = tmc22xx_pokstepper_enable,
    .set_speed = tmc22xx_pokstepper_set_speed,
};

#define TMC22XX_POKSTEPPER_DEFINE(inst)                                                            \
    static const struct tmc22xx_pokibot_config tmc22xx_pokibot_config_##inst = {                   \
        .nstdby = GPIO_DT_SPEC_INST_GET_OR(inst, nstdby_gpios, {0}),                               \
        .en = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                                       \
        .shared_uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                       \
        .address = DT_INST_PROP(inst, address),                                                    \
    };                                                                                             \
    static struct tmc22xx_pokibot_data tmc22xx_pokibot_data_##inst = {                             \
        .resolution = DT_INST_PROP(inst, micro_step_res),                                          \
        .irun = DT_INST_PROP(inst, irun),                                                          \
        .ihold = DT_INST_PROP(inst, ihold),                                                        \
        .iholddelay = DT_INST_PROP(inst, iholddelay),                                              \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, tmc22xx_pokstepper_init, NULL, &tmc22xx_pokibot_data_##inst,       \
                          &tmc22xx_pokibot_config_##inst, POST_KERNEL,                             \
                          CONFIG_POKSTEPPER_INIT_PRIORITY, &tmc22xx_pokstepper_api);

#define DT_DRV_COMPAT adi_tmc2209_pokibot
DT_INST_FOREACH_STATUS_OKAY(TMC22XX_POKSTEPPER_DEFINE)
#undef DT_DRV_COMPAT
