#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(main);

const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

int main(void)
{

    LOG_INF("trying can dev");
    if (!device_is_ready(can_dev)) {
        LOG_ERR("CAN: Device %s not ready.\n", can_dev->name);
        return 0;
    }
    LOG_INF("CAn dev ok");

    while (1) {
        k_sleep(K_MSEC(500));

    }
}
