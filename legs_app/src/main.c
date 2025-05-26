#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "control/control.h"
#include "com/com.h"

LOG_MODULE_REGISTER(main);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

void do_square(void) {
    control_set_pos((pos2_t){.a = 0, .x = 0, .y = 0});
    control_set_brake(false);

    pos2_t src[] = {
        (pos2_t){.a = 0.0f, .x = 200.0f, .y = 0.0f},
        (pos2_t){.a = 0.0f, .x = 200.0f, .y = 200.0f},
        (pos2_t){.a = 0.0f, .x = 0.0f, .y = 200.0f},
        (pos2_t){.a = 0.0f, .x = 0.0f, .y = 0.0f},
    };
    control_set_waypoints(src, ARRAY_SIZE(src));
}

int main(void)
{
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_HIGH);
    control_start();
    com_start();
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_HIGH);
    // _test_motor_cmd();
    // _test_calibration_angle();
    // do_square();
    return 0;
}
