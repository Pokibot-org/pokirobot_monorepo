#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/servomotor.h>
#include <pokibot/poktypes.h>

LOG_MODULE_REGISTER(main);

const struct device *servomotor0 = DEVICE_DT_GET(DT_NODELABEL(servomotor0));


int main(void)
{
    while (1) {
        servomotor_set_angle(servomotor0, 0);
        k_sleep(K_SECONDS(1));
        servomotor_set_angle(servomotor0, M_PI);
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
