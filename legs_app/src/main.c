#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "control/control.h"
#include "com/com.h"

LOG_MODULE_REGISTER(main);

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
    control_start();
    // com_start();
    // _test_motor_cmd();
    _test_calibration_angle();
    // do_square();
    return 0;
}
