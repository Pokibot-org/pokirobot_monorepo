#include "pakicontrol.h"
#include <stdint.h>
#include <zephyr/drivers/uart.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(rf_module))
const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(rf_module));

void pakicontrol_activate(void)
{
    uint8_t frame[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x44, 0x33};
    for (int i = 0; i < sizeof(frame); i++) {
        uart_poll_out(dev, frame[i]);
    }
}
#else
void pakicontrol_activate(void){}
#endif
