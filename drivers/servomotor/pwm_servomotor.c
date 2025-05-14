#define DT_DRV_COMPAT pokibot_pwm_servomotor
#include <pokibot/drivers/servomotor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(pwm_servomotor, CONFIG_SERVOMOTOR_LOG_LEVEL);

#define PWM_SERVO_RAD_TO_DEG(_rad) ((_rad) * 180.0f / 3.14159265358979323846f)

struct pwm_servomotor_cfg {
    struct pwm_dt_spec pwm;
    int min_pulse_ns;
    int max_pulse_ns;
    int min_angle;
    int max_angle;
};

struct pwm_servomotor_data {
};

static int pwm_servomotor_set_angle(const struct device *dev, float angle)
{
    const struct pwm_servomotor_cfg *cfg = dev->config;

    float angle_deg = PWM_SERVO_RAD_TO_DEG(angle);
    LOG_DBG("Target angle %.2f", (double)angle_deg);
    float angle_ratio =
        (angle_deg -cfg->min_angle) / abs(cfg->max_angle - cfg->min_angle);
    LOG_DBG("angle_ratio %.2f", (double)angle_ratio);
    uint32_t pulse = cfg->min_pulse_ns + angle_ratio * (cfg->max_pulse_ns - cfg->min_pulse_ns);
    LOG_DBG("PWM(%s) channel %d set %d/%d", cfg->pwm.dev->name, cfg->pwm.channel, pulse,
            cfg->pwm.period);
    return pwm_set_pulse_dt(&cfg->pwm, pulse);
}
static int pwm_servomotor_init(const struct device *dev)
{
    const struct pwm_servomotor_cfg *cfg = dev->config;
    if (!pwm_is_ready_dt(&cfg->pwm)) {
        return -1;
    }
    return 0;
}

static const struct servomotor_driver_api servomotor_api = {.set_angle = pwm_servomotor_set_angle};

#define SERVOMOTOR_INIT(n)                                                                         \
                                                                                                   \
    static struct pwm_servomotor_cfg servomotor_data_##n = {};                                     \
                                                                                                   \
    static const struct pwm_servomotor_cfg servomotor_cfg_##n = {                                  \
        .pwm = PWM_DT_SPEC_INST_GET(n),                                                            \
        .min_pulse_ns = DT_INST_PROP(n, min_pulse),                                                \
        .max_pulse_ns = DT_INST_PROP(n, max_pulse),                                                \
        .min_angle = DT_INST_PROP(n, min_angle),                                                   \
        .max_angle = DT_INST_PROP(n, max_angle),                                                   \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pwm_servomotor_init, NULL, &servomotor_data_##n, &servomotor_cfg_##n, \
                          POST_KERNEL, CONFIG_SERVOMOTOR_INIT_PRIORITY, &servomotor_api);

DT_INST_FOREACH_STATUS_OKAY(SERVOMOTOR_INIT)
