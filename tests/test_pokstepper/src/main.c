#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/pokstepper.h>

LOG_MODULE_REGISTER(main);

const struct device *stepper0 = DEVICE_DT_GET(DT_NODELABEL(stepper0));

int main(void)
{
    pokstepper_enable(stepper0, true);
    LOG_INF("Set speed +");
    pokstepper_set_speed(stepper0, 500);
    k_sleep(K_SECONDS(2));
    LOG_INF("Set speed -");
    pokstepper_set_speed(stepper0, -500);
    k_sleep(K_SECONDS(2));
    LOG_INF("Set speed 0");
    pokstepper_set_speed(stepper0, 0);
    pokstepper_enable(stepper0, false);
    k_sleep(K_FOREVER);
    return 0;
}
