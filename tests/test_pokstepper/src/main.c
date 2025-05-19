#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/pokstepper.h>

LOG_MODULE_REGISTER(main);

const struct device *stepper0 = DEVICE_DT_GET(DT_NODELABEL(stepper0));

int main(void)
{
    pokstepper_enable(stepper0, true);

    int ret = pokstepper_go_to_stall(stepper0, 500, 10000, 40);
    if (ret) {
        LOG_ERR("Go to stall failed error %d", ret);
    } else {
        int pos;
        pokstepper_get_pos(stepper0, &pos);
        LOG_ERR("Found stall at %d", pos);
    }

    int32_t speed = 10;
    int sleep_time = 1;
    for (int i=0; i < 4; i ++) {
        LOG_INF("Set speed +%d", speed);
        pokstepper_set_speed(stepper0, speed);
        k_sleep(K_SECONDS(sleep_time));
        LOG_INF("Set speed -%d", speed);
        pokstepper_set_speed(stepper0, -speed);
        k_sleep(K_SECONDS(sleep_time));
        speed *= 5;
    }
    LOG_INF("Set speed 0");
    pokstepper_set_speed(stepper0, 0);
    pokstepper_enable(stepper0, false);
    k_sleep(K_FOREVER);
    return 0;
}
