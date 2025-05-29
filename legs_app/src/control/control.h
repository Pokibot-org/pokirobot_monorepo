#ifndef CONTROL_H
#define CONTROL_H
#include <zephyr/kernel.h>
#include <pokibot/lib/pokutils.h>

int control_start(void);
int control_set_pos(pos2_t pos);
int control_get_pos(pos2_t *pos);
int control_get_dir(float *dir);
int control_set_brake(bool brake);
int control_set_waypoints(pos2_t *src, int n);
int control_set_planar_vmax(float vmax);
int control_set_angular_vmax(float vmax);

int control_task_wait_target(float planar_sensivity, float angular_sensivity,
                             uint32_t timeout_target_ms, uint32_t timeout_brake_ms);
#define control_task_wait_target_default(_timeout_target_ms, _timeout_brake_ms)                    \
    control_task_wait_target(CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT,                            \
                             CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT, _timeout_target_ms,       \
                             _timeout_brake_ms)


void _test_motor_cmd();
void _test_target();
void _test_calibration_distance();
void _test_calibration_angle();
void _test_calibration_mix();
void _test_connerie();
void _test_drawing();


#endif
