#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/drivers/pokmac.h>

LOG_MODULE_REGISTER(main);

const struct device *pokmac0 = DEVICE_DT_GET(DT_NODELABEL(pokmac0));
const struct device *pokmac1 = DEVICE_DT_GET(DT_NODELABEL(pokmac1));


void clbk0(uint8_t *payload_data, size_t payload_size)
{
    LOG_HEXDUMP_INF(payload_data, payload_size, "0: recv payload");
}

struct pokmac_recv_callback clbk0_struct = {.cb = clbk0};

int main(void)
{
    pokmac_register_recv_callback(pokmac0, &clbk0_struct);
    char msg[] = "HI!";
    LOG_INF("Sending msg unconfirmed");
    if (pokmac_send(pokmac1, msg, sizeof(msg), false)) {
        LOG_ERR("Error while sending message");
        return -1;
    }

    k_sleep(K_MSEC(10));

    LOG_INF("Sending msg confirmed");
    if (pokmac_send(pokmac1, msg, sizeof(msg), true)) {
        LOG_ERR("Error while sending message");
        return -1;
    } else {
        LOG_INF("Sending msg confirmed OK");
    }
    return 0;
}
