#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include <pokibot/drivers/pokstepper.h>
#include <pokibot/lib/pokutils.h>
#include "control.h"

LOG_MODULE_REGISTER(control);

#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))
#define CONTROL_WAYPOINTS_N   256

#define CONTROL_WAIT_OK             0
#define CONTROL_WAIT_TIMEOUT_TARGET (-1)
#define CONTROL_WAIT_TIMEOUT_BRAKE  (-2)

#define CONTROL_PERIOD_MS 2.0f
#define ROBOT_L           136.11984206494193f
#define WHEEL_PERIMETER   358.142f
#define MM_TO_USTEPS      100644.25490196078f

#define PLANAR_VMAX_DEFAULT   400.0f // 700 mm/s
#define PLANAR_VMAX           (obj.planar_vmax)
#define PLANAR_FACTOR         (0.06f * PLANAR_VMAX)
#define PLANAR_RAMP           (2.0f * PLANAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 2 seconds to reach vmax

#define ANGULAR_VMAX_DEFAULT   (0.7f * M_PI) // 0.5 rotation/s
#define ANGULAR_VMAX           (obj.angular_vmax)
#define ANGULAR_FACTOR         (0.7f * ANGULAR_VMAX)
#define ANGULAR_RAMP           (0.5f * ANGULAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 1 seconds to reach vmax

// normal
// #define CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT  5.0f             // 5mm
// #define CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f) // 3 deg
// #define WP_DIST_BIAS                               100.0f
// #define WP_SENSITIVITY                             300.0f
// drawing
#define CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT  5.0f
#define CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f)
#define WP_DIST_BIAS                               60.0f
#define WP_SENSITIVITY                             80.0f

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

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel);
vel2_t world_vel_from_delta2(pos2_t delta1, pos2_t delta2, vel2_t prev_vel);
vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel);
vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel);
omni3_t omni_from_local_vel(vel2_t local_vel);
vel2_t local_vel_from_omni(omni3_t omni);

struct control obj = {
    .stepper0 = DEVICE_DT_GET(DT_NODELABEL(stepper0)),
    .stepper1 = DEVICE_DT_GET(DT_NODELABEL(stepper1)),
    .stepper2 = DEVICE_DT_GET(DT_NODELABEL(stepper2)),
};
K_THREAD_STACK_DEFINE(control_thread_stack, 2048);
struct k_thread control_thread;

int control_set_pos(pos2_t pos)
{
    if (k_mutex_lock(&obj.access_mutex, K_FOREVER)) {
        return -ENOLCK;
    }
    obj.pos = pos;
    k_mutex_unlock(&obj.access_mutex);
    return 0;
}

int control_get_pos(pos2_t *pos)
{
    if (k_mutex_lock(&obj.access_mutex, K_FOREVER)) {
        return -ENOLCK;
    }
    *pos = obj.pos;
    k_mutex_unlock(&obj.access_mutex);
    return 0;
}

int control_get_dir(float *dir)
{
    *dir = obj.dir_angle;
    return 0;
}

static int control_get_pos_internal(pos2_t *pos)
{
    if (k_mutex_lock(&obj.access_mutex, K_MSEC(CONTROL_PERIOD_MS))) {
        LOG_ERR("Cant get pos internal");
        return -ENOLCK;
    }
    *pos = obj.pos;
    k_mutex_unlock(&obj.access_mutex);
    return 0;
}

int control_set_brake(bool brake)
{
    obj.brake = brake;
    return 0;
}

int control_set_waypoints(pos2_t *src, int n)
{
    int err = k_mutex_lock(&obj.waypoints.lock, CONTROL_MUTEX_TIMEOUT);
    if (!err) {
        obj.waypoints.idx = 0;
        obj.waypoints.n = n;
        memcpy(obj.waypoints.wps, src, n * sizeof(pos2_t));
        k_mutex_unlock(&obj.waypoints.lock);
    } else {
        LOG_ERR("could not lock waypoints mutex for write access");
        goto exit_error;
    }
    return 0;
exit_error:
    return -1;
}

int control_set_planar_vmax(float vmax)
{
    // TODO: mutex
    obj.planar_vmax = vmax;
    return 0;
}

int control_set_angular_vmax(float vmax)
{
    // TODO: mutex
    obj.angular_vmax = vmax;
    return 0;
}

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel)
{
    // planar speed capping + acceleration ramp
    float vx = PLANAR_FACTOR * NEG_SQRTF(delta.x);
    float vy = PLANAR_FACTOR * NEG_SQRTF(delta.y);
    const float planar_speed = sqrtf(vx * vx + vy * vy);
    const float planar_speed_prev = sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
    const float planar_speed_clamped =
        MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
    if (planar_speed > planar_speed_clamped) {
        const float planar_factor = planar_speed_clamped / planar_speed;
        vx *= planar_factor;
        vy *= planar_factor;
    }
    // angular speed capping + acceleration ramp
    const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
    float w =
        Z_CLAMP(ANGULAR_FACTOR * NEG_SQRTF(delta.a), MAX(-angular_speed_ramped, -ANGULAR_VMAX),
                MIN(angular_speed_ramped, ANGULAR_VMAX));
    // returning built vel
    vel2_t world_vel = {
        .vx = vx,
        .vy = vy,
        .w = w,
    };
    return world_vel;
}

vel2_t world_vel_from_delta2(pos2_t delta1, pos2_t delta2, vel2_t prev_vel)
{
    // compute biases
    float dist1 = sqrtf(delta1.x * delta1.x + delta1.y * delta1.y);
    float bias1 = MIN(dist1 / WP_DIST_BIAS, 1.0f);
    float bias2 = 1.0f - bias1;
    // planar speed capping + acceleration ramp
    float vx = PLANAR_FACTOR * (bias1 * NEG_SQRTF(delta1.x) + bias2 * NEG_SQRTF(delta2.x));
    float vy = PLANAR_FACTOR * (bias1 * NEG_SQRTF(delta1.y) + bias2 * NEG_SQRTF(delta2.y));
    const float planar_speed = sqrtf(vx * vx + vy * vy);
    const float planar_speed_prev = sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
    const float planar_speed_clamped =
        MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
    if (planar_speed > planar_speed_clamped) {
        const float planar_factor = planar_speed_clamped / planar_speed;
        vx *= planar_factor;
        vy *= planar_factor;
    }
    // angular speed capping + acceleration ramp
    const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
    float w =
        Z_CLAMP(ANGULAR_FACTOR * NEG_SQRTF(delta1.a), MAX(-angular_speed_ramped, -ANGULAR_VMAX),
                MIN(angular_speed_ramped, ANGULAR_VMAX));
    // returning built vel
    vel2_t world_vel = {
        .vx = vx,
        .vy = vy,
        .w = w,
    };
    return world_vel;
}

vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel)
{
    vel2_t world_vel = {
        .vx = cosf(pos.a) * local_vel.vx - sinf(pos.a) * local_vel.vy,
        .vy = sinf(pos.a) * local_vel.vx + cosf(pos.a) * local_vel.vy,
        .w = -local_vel.w,
    };
    return world_vel;
}

vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel)
{
    vel2_t local_vel = {
        .vx = cosf(pos.a) * world_vel.vx + sinf(pos.a) * world_vel.vy,
        .vy = -sinf(pos.a) * world_vel.vx + cosf(-pos.a) * world_vel.vy,
        .w = -world_vel.w,
    };
    return local_vel;
}

vel2_t local_vel_from_omni(omni3_t omni)
{
    vel2_t local_vel = {
        .vx = (2.0f * omni.v2 - omni.v1 - omni.v3) / 3.0f,
        .vy = M_SQRT3 * (omni.v3 - omni.v1) / 3.0f,
        .w = omni.v1 + omni.v2 + omni.v3 / (3.0f * ROBOT_L),
    };
    return local_vel;
}

omni3_t omni_from_local_vel(vel2_t local_vel)
{
    const float w_factor = ROBOT_L * local_vel.w;
    omni3_t omni3 = {
        .v1 = (-local_vel.vx - M_SQRT3 * local_vel.vy) / 2.0f + w_factor,
        .v2 = local_vel.vx + w_factor,
        .v3 = (-local_vel.vx + M_SQRT3 * local_vel.vy) / 2.0f + w_factor,
    };
    return omni3;
}

int control_task_wait_target(float planar_sensivity, float angular_sensivity,
                             uint32_t timeout_target_ms, uint32_t timeout_brake_ms)
{
    uint32_t target_timeout_cnt = 0;
    uint32_t brake_timeout_cnt = 0;
    obj.planar_target_sensivity = planar_sensivity;
    obj.angular_target_sensivity = angular_sensivity;
    obj.at_target = false;
    while (!obj.at_target) {
        target_timeout_cnt += 1;
        brake_timeout_cnt = obj.brake ? brake_timeout_cnt + 1 : 0;
        if (target_timeout_cnt > timeout_target_ms) {
            return CONTROL_WAIT_TIMEOUT_TARGET;
        }
        if (brake_timeout_cnt > timeout_brake_ms) {
            return CONTROL_WAIT_TIMEOUT_BRAKE;
        }
        k_sleep(K_MSEC(1));
    }
    return CONTROL_WAIT_OK;
}

static void control_task(void *arg0, void *arg1, void *arg2)
{
    LOG_INF("control task init");
    pos2_t pos = {.x = 0.0f, .y = 0.0f, .a = 0.0f};
    vel2_t world_vel = {.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
    vel2_t local_vel = {.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
    omni3_t motors_v = {.v1 = 0.0f, .v2 = 0.0f, .v3 = 0.0f};
    float wp_dist = -1.0f;
    float dist_prev = -1.0f;

    pokstepper_enable(obj.stepper0, true);
    pokstepper_enable(obj.stepper1, true);
    pokstepper_enable(obj.stepper2, true);

    bool wp_init = false;
    while (obj.ready) {
        bool skip_write = false;
        int n, idx;
        pos2_t wp1, wp2;
        // lock the waypoint structure
        int err = k_mutex_lock(&obj.waypoints.lock, K_MSEC((uint64_t)CONTROL_PERIOD_MS));
        if (err) {
            LOG_ERR("could not lock waypoints mutex for read access");
            skip_write = true;
            if (!wp_init) {
                goto continue_nocommit;
            }
        }
        // get next waypoints
        if (!skip_write) {
            wp_init = true;
            n = obj.waypoints.n;
            idx = obj.waypoints.idx;
            wp1 = obj.waypoints.wps[MIN(idx, n - 1)];
            wp2 = obj.waypoints.wps[MIN(idx + 1, n - 1)];
        }
        // update pos
        control_get_pos_internal(&pos);
        // control_get_motors_v(&motors_v);
        // local_vel = local_vel_from_omni(motors_v);
        // world_vel = world_vel_from_local(old_pos, local_vel);
        pos = (pos2_t){
            .x = pos.x + world_vel.vx * CONTROL_PERIOD_MS / 1000.0f,
            .y = pos.y + world_vel.vy * CONTROL_PERIOD_MS / 1000.0f,
            .a = pos.a + world_vel.w * CONTROL_PERIOD_MS / 1000.0f,
        };

        // update speed
        pos2_t delta1 = pos2_diff(wp1, pos);
        pos2_t delta2 = pos2_diff(wp2, pos);
        wp_dist = vec2_abs((vec2_t){delta1.x, delta1.y});
        if (idx >= (n - 1) && wp_dist < CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT &&
            delta2.a < CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT) {
            obj.at_target = true;
        } else {
            obj.at_target = false;
        }
        // world_vel = world_vel_from_delta(delta1, world_vel);
        world_vel = world_vel_from_delta2(delta1, delta2, world_vel);
        local_vel = local_vel_from_world(pos, world_vel);
        motors_v = omni_from_local_vel(local_vel);
        obj.dir_angle = atan2f(local_vel.vy, local_vel.vx);

        // brake if required
        if (obj.brake) {
            world_vel = (vel2_t){.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
            local_vel = (vel2_t){.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
            motors_v = (omni3_t){.v1 = 0.0f, .v2 = 0.0f, .v3 = 0.0f};
        }

        // update next waypoints
        if (!skip_write) {
            LOG_DBG("dist: %.2f | prev: %.2f | delta: %e", (double)wp_dist, (double)dist_prev,
                    (double)(wp_dist - dist_prev));
            if (idx < n && dist_prev >= 0.0f &&
                ((wp_dist > dist_prev && wp_dist <= WP_SENSITIVITY) ||
                 wp_dist < CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT)) {
                obj.waypoints.idx = MIN(obj.waypoints.idx + 1, obj.waypoints.n);
                dist_prev = -1.0f;
                LOG_DBG("idx: %d", obj.waypoints.idx);
            } else {
                dist_prev = wp_dist;
            }
            // release mutex and commit transaction
            k_mutex_unlock(&obj.waypoints.lock);
        }
        control_set_pos(pos);

        pokstepper_set_speed(obj.stepper0, (int32_t)(motors_v.v1 * MM_TO_USTEPS / WHEEL_PERIMETER));
        pokstepper_set_speed(obj.stepper1, (int32_t)(motors_v.v2 * MM_TO_USTEPS / WHEEL_PERIMETER));
        pokstepper_set_speed(obj.stepper2, (int32_t)(motors_v.v3 * MM_TO_USTEPS / WHEEL_PERIMETER));
    continue_nocommit:
        // sleep
        LOG_DBG("idx: %d", obj.waypoints.idx);
        LOG_DBG("pos: %.2f %.2f %.2f", (double)pos.x, (double)pos.y, (double)pos.a);
        LOG_DBG("target: %.2f %.2f %.2f", (double)wp1.x, (double)wp1.y, (double)wp1.a);
        LOG_DBG("next: %.2f %.2f %.2f", (double)wp2.x, (double)wp2.y, (double)wp2.a);
        LOG_DBG("speed: %.2f %.2f %.2f", (double)motors_v.v1, (double)motors_v.v2,
                (double)motors_v.v3);
        k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
    }
    LOG_INF("control task done");
}

int control_start(void)
{
    int ret = 0;
    if (k_mutex_init(&obj.access_mutex)) {
        LOG_ERR("Mutex init error access mutex");
        return -ENODEV;
    }

    if (k_mutex_init(&obj.waypoints.lock)) {
        LOG_ERR("Mutex init error waypoints lock");
        return -ENODEV;
    }

    if (!device_is_ready(obj.stepper0)) {
        LOG_ERR("Device stepper0 is not ready");
        return -ENODEV;
    }
    if (!device_is_ready(obj.stepper1)) {
        LOG_ERR("Device stepper1 is not ready");
        return -ENODEV;
    }
    if (!device_is_ready(obj.stepper2)) {
        LOG_ERR("Device stepper2 is not ready");
        return -ENODEV;
    }

    obj.brake = false;
    obj.at_target = false;
    obj.planar_target_sensivity = CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT;
    obj.angular_target_sensivity = CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT;
    obj.planar_vmax = PLANAR_VMAX_DEFAULT;
    obj.angular_vmax = ANGULAR_VMAX_DEFAULT;
    obj.dir_angle = 0.0f;
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_waypoints(&target, 1);
    obj.ready = true;

    k_tid_t thread_id = k_thread_create(&control_thread, control_thread_stack,
                                        K_THREAD_STACK_SIZEOF(control_thread_stack), control_task,
                                        NULL, NULL, NULL, 2, 0, K_NO_WAIT);
    k_thread_name_set(thread_id, "control");
    return ret;
}

void _test_motor_cmd()
{
    control_start();
    k_sleep(K_MSEC(1000));
    while (1) {
        pokstepper_set_speed(obj.stepper0, 0);
        pokstepper_set_speed(obj.stepper1, 0);
        pokstepper_set_speed(obj.stepper2, 0);
        k_sleep(K_MSEC(1000));
        pokstepper_set_speed(obj.stepper0, 10000);
        pokstepper_set_speed(obj.stepper1, 20000);
        pokstepper_set_speed(obj.stepper2, 40000);
        k_sleep(K_MSEC(1000));
        pokstepper_set_speed(obj.stepper0, 0);
        pokstepper_set_speed(obj.stepper1, 0);
        pokstepper_set_speed(obj.stepper2, 0);
        k_sleep(K_MSEC(1000));
        pokstepper_set_speed(obj.stepper0, 10000);
        pokstepper_set_speed(obj.stepper1, 0);
        pokstepper_set_speed(obj.stepper2, 0);
        k_sleep(K_MSEC(1000));
        pokstepper_set_speed(obj.stepper0, 0);
        pokstepper_set_speed(obj.stepper1, 10000);
        pokstepper_set_speed(obj.stepper2, 0);
        k_sleep(K_MSEC(1000));
        pokstepper_set_speed(obj.stepper0, 0);
        pokstepper_set_speed(obj.stepper1, 0);
        pokstepper_set_speed(obj.stepper2, 10000);
        k_sleep(K_MSEC(1000));
    }
}

void _test_target()
{
    LOG_INF("_test_target");
    control_start();
    pos2_t target;
    while (1) {
        target = (pos2_t){100.0f, 100.0f, 1.0f * M_PI};
        control_set_waypoints(&target, 1);
        LOG_DBG("1");
        k_sleep(K_MSEC(5000));
        target = (pos2_t){0.0f, 0.0f, 0.0f * M_PI};
        control_set_waypoints(&target, 1);
        LOG_DBG("2");
        k_sleep(K_MSEC(5000));
        target = (pos2_t){100.0f, -100.0f, -2.0f * M_PI};
        control_set_waypoints(&target, 1);
        LOG_DBG("3");
        k_sleep(K_MSEC(5000));
    }
}

void _test_calibration_distance()
{
    LOG_INF("_test_calibration");
    control_start();
    LOG_DBG("alive");
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
    k_sleep(K_MSEC(1000));
    target = (pos2_t){0.0f, 3000.0f, 0.0f * M_PI};
    control_set_waypoints(&target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
    k_sleep(K_MSEC(15000));
}

void _test_calibration_angle()
{
    LOG_INF("_test_calibration");
    control_start();
    LOG_DBG("alive");
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
    k_sleep(K_MSEC(1000));
    target = (pos2_t){0.0f, 0.0f, 20.0f * M_PI};
    control_set_waypoints(&target, 1);
    k_sleep(K_MSEC(15000));
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
}

void _test_calibration_mix()
{
    LOG_INF("_test_calibration");
    control_start();
    LOG_DBG("alive");
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
    k_sleep(K_MSEC(1000));
    target = (pos2_t){0.0f, 0.0f, 1.0f * M_PI};
    control_set_waypoints(&target, 1);
    k_sleep(K_MSEC(5000));
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
    target = (pos2_t){0.0f, 1000.0f, -1.0f * M_PI};
    control_set_waypoints(&target, 1);
    k_sleep(K_MSEC(5000));
    LOG_DBG("pos: %.2f %.2f %.2f", (double)obj.pos.x, (double)obj.pos.y, (double)obj.pos.a);
    LOG_DBG("target: %.2f %.2f %.2f", (double)obj.waypoints.wps[0].x,
            (double)obj.waypoints.wps[0].y, (double)obj.waypoints.wps[0].a);
}

void _test_connerie()
{
    LOG_INF("_test_connerie");
    control_start();
    LOG_DBG("alive");
    pos2_t target = (pos2_t){0.0f, 0.0f, 100.0f};
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&target, 1);
    k_sleep(K_MSEC(15000));
    obj.brake = true;
}

void _test_drawing()
{
    LOG_INF("_test_drawing");
    control_start();
    LOG_DBG("alive");
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&target, 1);
    k_sleep(K_MSEC(1000));

    // start of drawing
    pos2_t draw_wps[] = {
        (pos2_t){.x = 46.0f, .y = 202.0f, .a = 0.0f},
        (pos2_t){.x = 44.0f, .y = 202.0f, .a = 0.0f},
        (pos2_t){.x = 40.0f, .y = 199.0f, .a = 0.0f},
        (pos2_t){.x = 36.0f, .y = 195.0f, .a = 0.0f},
        (pos2_t){.x = 34.0f, .y = 190.0f, .a = 0.0f},
        (pos2_t){.x = 34.0f, .y = 190.0f, .a = 0.0f},
        (pos2_t){.x = 34.0f, .y = 168.0f, .a = 0.0f},
        (pos2_t){.x = 26.0f, .y = 161.0f, .a = 0.0f},
        (pos2_t){.x = 19.0f, .y = 154.0f, .a = 0.0f},
        (pos2_t){.x = 13.0f, .y = 146.0f, .a = 0.0f},
        (pos2_t){.x = 8.0f, .y = 138.0f, .a = 0.0f},
        (pos2_t){.x = 5.0f, .y = 129.0f, .a = 0.0f},
        (pos2_t){.x = 2.0f, .y = 119.0f, .a = 0.0f},
        (pos2_t){.x = 0.0f, .y = 109.0f, .a = 0.0f},
        (pos2_t){.x = 0.0f, .y = 98.0f, .a = 0.0f},
        (pos2_t){.x = 0.0f, .y = 98.0f, .a = 0.0f},
        (pos2_t){.x = 0.0f, .y = 98.0f, .a = 0.0f},
        (pos2_t){.x = 0.0f, .y = 89.0f, .a = 0.0f},
        (pos2_t){.x = 1.0f, .y = 80.0f, .a = 0.0f},
        (pos2_t){.x = 3.0f, .y = 71.0f, .a = 0.0f},
        (pos2_t){.x = 6.0f, .y = 61.0f, .a = 0.0f},
        (pos2_t){.x = 12.0f, .y = 52.0f, .a = 0.0f},
        (pos2_t){.x = 18.0f, .y = 43.0f, .a = 0.0f},
        (pos2_t){.x = 27.0f, .y = 34.0f, .a = 0.0f},
        (pos2_t){.x = 38.0f, .y = 24.0f, .a = 0.0f},
        (pos2_t){.x = 38.0f, .y = 12.0f, .a = 0.0f},
        (pos2_t){.x = 38.0f, .y = 12.0f, .a = 0.0f},
        (pos2_t){.x = 40.0f, .y = 7.0f, .a = 0.0f},
        (pos2_t){.x = 42.0f, .y = 3.0f, .a = 0.0f},
        (pos2_t){.x = 47.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 47.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 47.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 51.0f, .y = 1.0f, .a = 0.0f},
        (pos2_t){.x = 53.0f, .y = 2.0f, .a = 0.0f},
        (pos2_t){.x = 55.0f, .y = 4.0f, .a = 0.0f},
        (pos2_t){.x = 56.0f, .y = 6.0f, .a = 0.0f},
        (pos2_t){.x = 57.0f, .y = 10.0f, .a = 0.0f},
        (pos2_t){.x = 58.0f, .y = 14.0f, .a = 0.0f},
        (pos2_t){.x = 63.0f, .y = 12.0f, .a = 0.0f},
        (pos2_t){.x = 70.0f, .y = 10.0f, .a = 0.0f},
        (pos2_t){.x = 79.0f, .y = 9.0f, .a = 0.0f},
        (pos2_t){.x = 88.0f, .y = 9.0f, .a = 0.0f},
        (pos2_t){.x = 95.0f, .y = 8.0f, .a = 0.0f},
        (pos2_t){.x = 103.0f, .y = 9.0f, .a = 0.0f},
        (pos2_t){.x = 110.0f, .y = 11.0f, .a = 0.0f},
        (pos2_t){.x = 117.0f, .y = 14.0f, .a = 0.0f},
        (pos2_t){.x = 119.0f, .y = 7.0f, .a = 0.0f},
        (pos2_t){.x = 121.0f, .y = 3.0f, .a = 0.0f},
        (pos2_t){.x = 124.0f, .y = 1.0f, .a = 0.0f},
        (pos2_t){.x = 127.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 127.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 127.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 130.0f, .y = 0.0f, .a = 0.0f},
        (pos2_t){.x = 132.0f, .y = 1.0f, .a = 0.0f},
        (pos2_t){.x = 135.0f, .y = 4.0f, .a = 0.0f},
        (pos2_t){.x = 138.0f, .y = 11.0f, .a = 0.0f},
        (pos2_t){.x = 138.0f, .y = 11.0f, .a = 0.0f},
        (pos2_t){.x = 137.0f, .y = 25.0f, .a = 0.0f},
        (pos2_t){.x = 151.0f, .y = 37.0f, .a = 0.0f},
        (pos2_t){.x = 157.0f, .y = 44.0f, .a = 0.0f},
        (pos2_t){.x = 163.0f, .y = 52.0f, .a = 0.0f},
        (pos2_t){.x = 169.0f, .y = 61.0f, .a = 0.0f},
        (pos2_t){.x = 173.0f, .y = 72.0f, .a = 0.0f},
        (pos2_t){.x = 174.0f, .y = 77.0f, .a = 0.0f},
        (pos2_t){.x = 176.0f, .y = 84.0f, .a = 0.0f},
        (pos2_t){.x = 176.0f, .y = 90.0f, .a = 0.0f},
        (pos2_t){.x = 177.0f, .y = 97.0f, .a = 0.0f},
        (pos2_t){.x = 177.0f, .y = 97.0f, .a = 0.0f},
        (pos2_t){.x = 177.0f, .y = 97.0f, .a = 0.0f},
        (pos2_t){.x = 176.0f, .y = 108.0f, .a = 0.0f},
        (pos2_t){.x = 174.0f, .y = 118.0f, .a = 0.0f},
        (pos2_t){.x = 171.0f, .y = 128.0f, .a = 0.0f},
        (pos2_t){.x = 167.0f, .y = 137.0f, .a = 0.0f},
        (pos2_t){.x = 163.0f, .y = 145.0f, .a = 0.0f},
        (pos2_t){.x = 157.0f, .y = 153.0f, .a = 0.0f},
        (pos2_t){.x = 150.0f, .y = 161.0f, .a = 0.0f},
        (pos2_t){.x = 142.0f, .y = 168.0f, .a = 0.0f},
        (pos2_t){.x = 142.0f, .y = 189.0f, .a = 0.0f},
        (pos2_t){.x = 142.0f, .y = 189.0f, .a = 0.0f},
        (pos2_t){.x = 140.0f, .y = 194.0f, .a = 0.0f},
        (pos2_t){.x = 138.0f, .y = 199.0f, .a = 0.0f},
        (pos2_t){.x = 134.0f, .y = 202.0f, .a = 0.0f},
        (pos2_t){.x = 129.0f, .y = 204.0f, .a = 0.0f},
        (pos2_t){.x = 129.0f, .y = 204.0f, .a = 0.0f},
        (pos2_t){.x = 129.0f, .y = 204.0f, .a = 0.0f},
        (pos2_t){.x = 125.0f, .y = 203.0f, .a = 0.0f},
        (pos2_t){.x = 122.0f, .y = 202.0f, .a = 0.0f},
        (pos2_t){.x = 119.0f, .y = 200.0f, .a = 0.0f},
        (pos2_t){.x = 117.0f, .y = 197.0f, .a = 0.0f},
        (pos2_t){.x = 116.0f, .y = 194.0f, .a = 0.0f},
        (pos2_t){.x = 114.0f, .y = 191.0f, .a = 0.0f},
        (pos2_t){.x = 113.0f, .y = 183.0f, .a = 0.0f},
        (pos2_t){.x = 107.0f, .y = 184.0f, .a = 0.0f},
        (pos2_t){.x = 101.0f, .y = 185.0f, .a = 0.0f},
        (pos2_t){.x = 95.0f, .y = 186.0f, .a = 0.0f},
        (pos2_t){.x = 89.0f, .y = 186.0f, .a = 0.0f},
        (pos2_t){.x = 76.0f, .y = 185.0f, .a = 0.0f},
        (pos2_t){.x = 63.0f, .y = 183.0f, .a = 0.0f},
        (pos2_t){.x = 62.0f, .y = 189.0f, .a = 0.0f},
        (pos2_t){.x = 60.0f, .y = 195.0f, .a = 0.0f},
        (pos2_t){.x = 59.0f, .y = 197.0f, .a = 0.0f},
        (pos2_t){.x = 57.0f, .y = 200.0f, .a = 0.0f},
        (pos2_t){.x = 53.0f, .y = 202.0f, .a = 0.0f},
        (pos2_t){.x = 49.0f, .y = 202.0f, .a = 0.0f},
    };
    int draw_wps_len = 104;
    // end of drawing
    // scale
    float scaling = 3.0f;
    for (int i = 0; i < draw_wps_len; i++) {
        draw_wps[i].x *= scaling;
        draw_wps[i].y *= scaling;
        LOG_DBG("p[%d]: %.2f %.2f %.2f", i, (double)draw_wps[i].x, (double)draw_wps[i].y,
                (double)draw_wps[i].a);
    }
    control_set_waypoints(draw_wps, draw_wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);
}
