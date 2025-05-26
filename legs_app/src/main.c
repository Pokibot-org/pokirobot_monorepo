#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "control/control.h"
#include "com/com.h"

LOG_MODULE_REGISTER(main);

int main(void)
{
    control_start();
    // com_start();
    // _test_motor_cmd();
    _test_calibration_angle();

    k_sleep(K_FOREVER);
    return 0;
}
