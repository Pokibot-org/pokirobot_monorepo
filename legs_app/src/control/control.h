#ifndef CONTROL_H
#define CONTROL_H
#include <zephyr/kernel.h>
#include <pokibot/lib/pokutils.h>

#define CONTROL_WAYPOINTS_N 256

typedef struct waypoints {
    pos2_t wps[CONTROL_WAYPOINTS_N];
    int n;
    int idx;
    struct k_mutex lock;
} waypoints_t;

typedef struct omni3 {
    float v1;
    float v2;
    float v3;
} omni3_t;

struct control {
    bool brake;
    bool ready;
    bool at_target;
    float planar_target_sensivity;
    float angular_target_sensivity;
    float dir_angle;
    float planar_vmax;
    float angular_vmax;
    pos2_t pos;
    waypoints_t waypoints;
    const struct device *stepper0;
    const struct device *stepper1;
    const struct device *stepper2;
    struct k_mutex access_mutex;
};

int control_start(struct control *obj);
int control_set_pos(struct control *obj, pos2_t pos);
int control_get_pos(struct control *obj, pos2_t *pos);
int control_get_dir(struct control *obj, float *dir);
int control_set_brake(struct control *obj, bool brake);
int control_set_waypoints(struct control *obj, pos2_t *src, int n);
int control_set_planar_vmax(struct control *obj, float vmax);
int control_set_angular_vmax(struct control *obj, float vmax);

int control_task_wait_target(struct control *obj, float planar_sensivity, float angular_sensivity,
                             uint32_t timeout_target_ms, uint32_t timeout_brake_ms);
#define control_task_wait_target_default(_obj, _timeout_target_ms, _timeout_brake_ms)              \
    control_task_wait_target(_obj, CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT,                      \
                             CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT, _timeout_target_ms,       \
                             _timeout_brake_ms)

void _test_motor_cmd(struct control *obj);
void _test_target(struct control *obj);
void _test_calibration_distance(struct control *obj);
void _test_calibration_angle(struct control *obj);
void _test_calibration_mix(struct control *obj);
void _test_connerie(struct control *obj);
void _test_drawing(struct control *obj);

#endif
