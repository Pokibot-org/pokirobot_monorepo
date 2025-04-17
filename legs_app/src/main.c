#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "control/control.h"

LOG_MODULE_REGISTER(main);

int main(void)
{
    control_set_pos((pos2_t){.a = 0, .x = 0, .y = 0});
    control_set_brake(false);
    pos2_t src[] = {
        (pos2_t){.a = 0.0f, .x = 1.0f, .y = 0.0f},
        (pos2_t){.a = 0.0f, .x = 1.0f, .y = 1.0f},
        (pos2_t){.a = 0.0f, .x = 0.0f, .y = 1.0f},
        (pos2_t){.a = 0.0f, .x = 0.0f, .y = 0.0f},
    };
    control_set_waypoints(src, ARRAY_SIZE(src));
    k_sleep(K_FOREVER);
    return 0;
}
