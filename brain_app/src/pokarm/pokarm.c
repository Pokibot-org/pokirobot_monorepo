#include <zephyr/kernel.h>
#include <pokibot/drivers/pokstepper.h>
#include "pokarm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pokarm);

const struct device *stepper = DEVICE_DT_GET(DT_NODELABEL(stepper_arm));
const struct device *servo_deploy = DEVICE_DT_GET(DT_NODELABEL(servo_deploy_arm));
const struct device *servo_left = DEVICE_DT_GET(DT_NODELABEL(servo_left_gripper));
const struct device *servo_right = DEVICE_DT_GET(DT_NODELABEL(servo_right_gripper));


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
