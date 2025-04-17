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
#define ROBOT_L           160.404f
#define WHEEL_PERIMETER   358.142f
#define MM_TO_USTEPS      102657.14f

#define PLANAR_VMAX   400.0f // 700 mm/s
#define PLANAR_FACTOR (0.06f * PLANAR_VMAX)
#define PLANAR_RAMP   (2.0f * PLANAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 2 seconds to reach vmax

#define ANGULAR_VMAX   (0.7f * M_PI) // 0.5 rotation/s
#define ANGULAR_FACTOR (0.7f * ANGULAR_VMAX)
#define ANGULAR_RAMP   (0.5f * ANGULAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 1 seconds to reach vmax

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
    bool start;
    bool start_init;
    bool brake;
    bool ready;
    bool at_target;
    float planar_target_sensivity;
    float angular_target_sensivity;
    float dir_angle;
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
        // control_get_motors_v(&shared_ctrl, &motors_v);
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
                obj.waypoints.idx =
                    MIN(obj.waypoints.idx + 1, obj.waypoints.n);
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

int control_init(void)
{
    int ret = 0;
    if (!k_mutex_init(&obj.access_mutex)){
        LOG_ERR("Mutex init error");
        return -ENODEV;
    }

    if (!k_mutex_init(&obj.waypoints.lock)){
        LOG_ERR("Mutex init error");
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

    obj.start = false;
    obj.brake = false;
    obj.at_target = false;
    obj.planar_target_sensivity = CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT;
    obj.angular_target_sensivity = CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT;
    obj.dir_angle = 0.0f;
    control_set_pos((pos2_t){0.0f, 0.0f, 0.0f});
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_waypoints(&target, 1);
    obj.ready = true;

   	k_tid_t thread_id = k_thread_create(&control_thread, control_thread_stack, K_THREAD_STACK_SIZEOF(control_thread_stack),
										control_task, NULL, NULL, NULL, 2, 0, K_NO_WAIT);
	k_thread_name_set(thread_id, "control");
    return ret;
}

SYS_INIT(control_init, APPLICATION, 50);
