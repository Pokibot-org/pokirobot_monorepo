#include <zephyr/kernel.h>
#include <pokibot/drivers/pokstepper.h>
#include "pokarm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pokarm);

const struct device *stepper_dev = DEVICE_DT_GET(DT_NODELABEL(stepper_arm));


void pokarm_reset_pos(void) {

}

void pokarm_deploy(bool state)
{

}

void pokarm_pinch(bool state)
{

}

void pokarm_lift(bool state)
{

}
