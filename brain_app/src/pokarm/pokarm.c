#include <zephyr/kernel.h>
#include <pokibot/drivers/pokstepper.h>
#include <pokibot/drivers/servomotor.h>
#include <pokibot/lib/pokutils.h>
#include "pokarm.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pokarm);

#define MM_TO_STEP(x) ((200 * (x)) / 30)

const int32_t rail_len_mm = 200;
const int32_t ustep_res = 16;
const int32_t nominal_speed = ustep_res * 400;

const struct device *stepper = DEVICE_DT_GET(DT_NODELABEL(stepper_arm));
const struct device *servo_deploy = DEVICE_DT_GET(DT_NODELABEL(servo_deploy_arm));
const struct device *servo_left = DEVICE_DT_GET(DT_NODELABEL(servo_left_gripper));
const struct device *servo_right = DEVICE_DT_GET(DT_NODELABEL(servo_right_gripper));
const struct device *servo_banner = DEVICE_DT_GET(DT_NODELABEL(servo_banner));


void banner_deploy(bool state) {
    LOG_INF("Banner deploy %d", state);
    if (state) {
        servomotor_set_angle(servo_banner, DEG_TO_RAD(180));
    } else {
        servomotor_set_angle(servo_banner, DEG_TO_RAD(100));
    }
}

void pokarm_reset_pos(void) {
    pokstepper_enable(stepper, false);
    k_sleep(K_MSEC(100));
    pokstepper_enable(stepper, true);
}

void pokarm_deploy(bool state)
{
    LOG_INF("Pokarm deploy %d", state);
    if (state) {
        servomotor_set_angle(servo_deploy, DEG_TO_RAD(24));
    } else {
        servomotor_set_angle(servo_deploy, DEG_TO_RAD(85));
    }
}

void pokarm_pinch(bool state)
{
    const float full_closed_servo_pos_left = DEG_TO_RAD(0);
    const float full_closed_servo_pos_right = DEG_TO_RAD(180);
    float open_angle = DEG_TO_RAD(27);
    float pinch_angle = DEG_TO_RAD(19);

    LOG_INF("Pokarm pinch %d", state);

    if (state) {
        servomotor_set_angle(servo_left, full_closed_servo_pos_left + pinch_angle);
        servomotor_set_angle(servo_right, full_closed_servo_pos_right - pinch_angle);
    } else {
        servomotor_set_angle(servo_left, full_closed_servo_pos_left + open_angle);
        servomotor_set_angle(servo_right, full_closed_servo_pos_right - open_angle);
    }
}

void pokarm_lift(bool state)
{
    int err = 0;
    LOG_INF("Pokarm lift %d", state);
    if (true) {
        err = pokstepper_move_to(stepper, nominal_speed, -MM_TO_STEP(70));
    } else {
        err = pokstepper_move_to(stepper, nominal_speed, -MM_TO_STEP(0));
    }

    if (err) {
        LOG_ERR("ERR %d", err);
    }
}
