#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include "control/control.h"
#include "com/com.h"

LOG_MODULE_REGISTER(main);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

int main(void)
{
    static struct control control_obj = {
        .stepper0 = DEVICE_DT_GET(DT_NODELABEL(stepper0)),
        .stepper1 = DEVICE_DT_GET(DT_NODELABEL(stepper1)),
        .stepper2 = DEVICE_DT_GET(DT_NODELABEL(stepper2)),
    };
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_HIGH);
    control_start(&control_obj);
    com_start(&control_obj);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_HIGH);

    // _test_square(&control_obj);
    // _test_motor_cmd(&control_obj);
    // _test_calibration_angle(&control_obj);
    return 0;
}
