#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <pokibot/poklegscom.h>
#include <pokibot/lib/pokutils.h>
#include <pokibot/poktypes.h>
#include <pokibot/drivers/lidar.h>
#include "nav.h"
#include "pokdefs.h"
#include "astar-jps/AStar.h"
#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include <stddef.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nav, CONFIG_NAV_LOG_LEVEL);

enum nav_mode {
    NAV_MODE_DIRECT,
    NAV_MODE_WITH_PATHFINDING
};

enum nav_mode current_mode = NAV_MODE_DIRECT;

const struct device *lidar_dev = DEVICE_DT_GET(DT_ALIAS(mainlidar));

K_EVENT_DEFINE(nav_events);
K_THREAD_STACK_DEFINE(nav_workq_stack, 2048);
static struct k_work_q nav_workq;

static void check_target_reached_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(check_target_reached_work, check_target_reached_work_handler);

// TODO: can be transformed in a hander for all nav events with a proper state machine
static void recompute_path_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(recompute_path_work, recompute_path_work_handler);

static void timeout_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(timeout_work, timeout_work_handler);

static bool current_obstacle_detected_state = false;
static pos2_t target_pos;
bool had_a_target_pos = false;

#define ASTAR_NODE_EMPTY    1
#define ASTAR_NODE_FULL     0
#define GRID_REZ    ((int)(100 / 5))
#define GRID_SIZE_Y (2 * GRID_REZ + 1)
#define GRID_SIZE_X (3 * GRID_REZ + 1)
#define BOARD_BORDER_MARGIN ROBOT_RADIUS
#define COLLISION_MARGIN    0.05f
#define OBSTACLE_MARGIN     (ROBOT_RADIUS + OPPONENT_ROBOT_MAX_RADIUS + COLLISION_MARGIN)

#define BOARD_TO_ASTAR_GRID_DIM(x) ((x) * (GRID_SIZE_X - 1) / (BOARD_SIZE_X - 2 * BOARD_BORDER_MARGIN))

#define BOARD_TO_ASTAR_GRID_X(x)                                                                   \
    (MIN(MAX((int)((x - BOARD_MIN_X - BOARD_BORDER_MARGIN) * (GRID_SIZE_X - 1) /                   \
                   (BOARD_SIZE_X - 2 * BOARD_BORDER_MARGIN)),                                      \
             0),                                                                                   \
         GRID_SIZE_X - 1))
#define BOARD_TO_ASTAR_GRID_Y(y)                                                                   \
    (MIN(MAX((int)((y - BOARD_MIN_Y - BOARD_BORDER_MARGIN) * (GRID_SIZE_Y - 1) /                   \
                   (BOARD_SIZE_Y - 2 * BOARD_BORDER_MARGIN)),                                      \
             0),                                                                                   \
         GRID_SIZE_Y - 1))
#define ASTAR_GRID_TO_BOARD_X(x)                                                                   \
    ((float)x * (BOARD_SIZE_X - 2 * BOARD_BORDER_MARGIN) / (GRID_SIZE_X - 1) + BOARD_MIN_X +       \
     BOARD_BORDER_MARGIN)
#define ASTAR_GRID_TO_BOARD_Y(y)                                                                   \
    ((float)y * (BOARD_SIZE_Y - 2 * BOARD_BORDER_MARGIN) / (GRID_SIZE_Y - 1) + BOARD_MIN_Y +       \
     BOARD_BORDER_MARGIN)

static char astar_grids[2][GRID_SIZE_Y * GRID_SIZE_X];
static int current_astar_grid = 0;
static char in_use_astar_grid[GRID_SIZE_Y * GRID_SIZE_X];
static char static_astar_grid[GRID_SIZE_Y * GRID_SIZE_X];

static const pos2_t sensivity = {
    .x = 0.01f,
    .y = 0.01f,
    .a = DEG_TO_RAD(2.0f),
};

static void astar_mark_grid(char *grid, int x, int y, char value)
{
    // Ensure the coordinates are within valid grid bounds
    if (x < 0 || x >= GRID_SIZE_X || y < 0 || y >= GRID_SIZE_Y) {
        return; // Avoid out-of-bounds memory access
    }

    // Get the 1D index for the 2D grid coordinates
    int grid_index = astar_getIndexByWidth(GRID_SIZE_X, x, y);

    // Mark the grid cell as occupied
    grid[grid_index] = value;
}

static void astar_grid_flood_fill_circle(char *grid, int cx, int cy, int r, char value)
{
    for (int x = -r; x <= r; x++) {
        for (int y = -r; y <= r; y++) {
            if (x * x + y * y <= r * r) { // Check if inside the circle
                astar_mark_grid(grid, cx + x, cy + y, value);
            }
        }
    }
}

static void astar_grid_flood_fill_rectangle_with_margin(char *grid, int cx, int cy, int width, int height, int margin, char value)
{
    astar_grid_flood_fill_circle(grid, cx - width / 2, cy - height / 2, margin, value);
    astar_grid_flood_fill_circle(grid, cx - width / 2, cy + height / 2, margin, value);
    astar_grid_flood_fill_circle(grid, cx + width / 2, cy - height / 2, margin, value);
    astar_grid_flood_fill_circle(grid, cx + width / 2, cy + height / 2, margin, value);

    int width_w_margin = width + margin * 2;
    int height_w_margin = height + margin * 2;
    for (int x = -width_w_margin / 2 ; x <= width_w_margin / 2; x++) {
        for (int y = -height_w_margin / 2; y <= height_w_margin / 2; y++) {
            if ((x < -width / 2 || x > width / 2) && (y < -height / 2 || y > height / 2)) {
                continue;
            }
            int gx = cx + x;
            int gy = cy + y;
            astar_mark_grid(grid, gx, gy, value);
        }
    }
}

static void log_astar_grid(char *current_grid, int start, int end) {
    pos2_t robot_pos;
    poklegscom_get_pos(&robot_pos);
    char line[GRID_SIZE_X * 2 + 1];
    memset(line, ' ', sizeof(line));
    line[GRID_SIZE_X * 2] = '\0';
    LOG_INF("A* grid:");
    for (int y = 0; y < GRID_SIZE_Y; y++) {
        for (int x = 0; x < GRID_SIZE_X; x++) {
            int index = astar_getIndexByWidth(GRID_SIZE_X, x, GRID_SIZE_Y - y - 1);
            if (start == index) {
                line[x*2] = 'S';
            } else if (end == index) {
                line[x*2] = 'E';
            } else if (current_grid[index] == ASTAR_NODE_FULL) {
                line[x*2] = '1';
            } else {
                line[x * 2] = '0';
            }
        }
        LOG_INF("%s", line);
    }
}

int nav_set_pos(const pos2_t *pos)
{
    return poklegscom_set_pos(pos);
}

int nav_set_break(bool status)
{
    return poklegscom_set_break(status);
}

static int go_to_with_pathfinding(char* astar_grid, const pos2_t *start_pos, const pos2_t *target_pos)
{
    uint32_t start_time = k_uptime_get_32();
    int sol_length;

    LOG_INF("from{.x=%.2f, .y=%.2f}", (double)start_pos->x, (double)start_pos->y);
    LOG_INF("to{.x=%.2f, .y=%.2f}", (double)target_pos->x, (double)target_pos->y);
    int start = astar_getIndexByWidth(GRID_SIZE_X, BOARD_TO_ASTAR_GRID_X(start_pos->x),
                                      BOARD_TO_ASTAR_GRID_Y(start_pos->y));
    int end = astar_getIndexByWidth(GRID_SIZE_X, BOARD_TO_ASTAR_GRID_X(target_pos->x),
                                    BOARD_TO_ASTAR_GRID_Y(target_pos->y));
    LOG_INF("Indexes start %d | end %d", start, end);

    int *indexes =
        astar_compute(astar_grid, &sol_length, GRID_SIZE_X, GRID_SIZE_Y, start, end);
    LOG_INF("Compute A* in %dms", k_uptime_get_32() - start_time);
    LOG_INF("Sol len %d | indexes %p", sol_length, (void *)indexes);
    if (!indexes) {
        LOG_ERR("Error in astar_compute");
        log_astar_grid(astar_grid, start, end);
        return -1;
    }
    if (sol_length == 0) {
        LOG_WRN("Already at destination");
        if (indexes) {
            k_free(indexes);
        }
        poklegscom_set_waypoints(target_pos, 1);
        return 0;
    }
    pos2_t *wps = k_malloc(sizeof(pos2_t) * sol_length);
    if (!wps) {
        LOG_ERR("wps malloc failed");
        k_free(indexes);
        return -2;
    }
    for (int i = 0; i < sol_length; i++) {
        int x, y;
        int node_index = sol_length - i - 1;
        astar_getCoordByWidth(GRID_SIZE_X, indexes[i], &x, &y);
        wps[node_index].a = start_pos->a;
        wps[node_index].x = ASTAR_GRID_TO_BOARD_X(x);
        wps[node_index].y = ASTAR_GRID_TO_BOARD_Y(y);
    }
    k_free(indexes);
    LOG_INF("First wp: {.x=%0.3f, .y=%0.3f}", (double)wps[0].x, (double)wps[0].y);
    wps[sol_length - 1] = *target_pos;
    poklegscom_set_waypoints(wps, sol_length);
    k_free(wps);
    LOG_INF("go_to_with_pathfinding in %dms", k_uptime_get_32() - start_time);
    return 0;
}

static int go_to_with_pathfinding_easy(const pos2_t *target_pos)
{
    pos2_t robot_pos;
    poklegscom_get_pos(&robot_pos);
    int start = astar_getIndexByWidth(GRID_SIZE_X, BOARD_TO_ASTAR_GRID_X(robot_pos.x),
                                      BOARD_TO_ASTAR_GRID_Y(robot_pos.y));
    memcpy(in_use_astar_grid, astar_grids[current_astar_grid], sizeof(in_use_astar_grid));
    in_use_astar_grid[start] = ASTAR_NODE_EMPTY;
    return go_to_with_pathfinding(in_use_astar_grid, &robot_pos, target_pos);
}

static void nav_stop_all(void)
{
    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&check_target_reached_work, &sync);
    k_work_cancel_delayable_sync(&recompute_path_work, &sync);
    k_work_cancel_delayable_sync(&timeout_work, &sync);
    had_a_target_pos = false;
}

int nav_go_to(const pos2_t *pos, k_timeout_t timeout)
{
    LOG_DBG(LOG_POS_ARGS("target", *pos));
    nav_stop_all();
    k_event_clear(&nav_events, 0xFFFFFFFF);
    target_pos = *pos;
    had_a_target_pos = true;
    current_mode = NAV_MODE_WITH_PATHFINDING;
    if (!current_obstacle_detected_state) {
        poklegscom_set_break(0);
    }
    go_to_with_pathfinding_easy(pos);
    k_work_schedule_for_queue(&nav_workq, &recompute_path_work, K_MSEC(1000));
    k_work_schedule_for_queue(&nav_workq, &check_target_reached_work, K_NO_WAIT);
    k_work_schedule_for_queue(&nav_workq, &timeout_work, timeout);
    return 0;
}

int nav_go_to_direct(const pos2_t *pos, k_timeout_t timeout)
{
    LOG_DBG(LOG_POS_ARGS("target", *pos));
    nav_stop_all();
    k_event_clear(&nav_events, 0xFFFFFFFF);
    target_pos = *pos;
    had_a_target_pos = true;
    current_mode = NAV_MODE_DIRECT;
    if (!current_obstacle_detected_state) {
        poklegscom_set_break(0);
    }
    poklegscom_set_waypoints(pos, 1);
    k_work_schedule_for_queue(&nav_workq, &check_target_reached_work, K_NO_WAIT);
    k_work_schedule_for_queue(&nav_workq, &timeout_work, timeout);
    return 0;
}

void nav_cancel(void)
{
    nav_stop_all();
    poklegscom_set_break(1);
    k_event_set(&nav_events, NAV_EVENT_CANCELED);
}

void nav_wait_events(uint32_t *events)
{
    k_event_wait(&nav_events, 0xFFFFFFFF, false, K_FOREVER);
}

static void check_target_reached_work_handler(struct k_work *work)
{
    pos2_t current_pos;
    poklegscom_get_pos(&current_pos);
    pos2_t diff = pos2_abs(pos2_diff(current_pos, target_pos));
    if (POS2_COMPARE(diff, <, sensivity)) {
        LOG_INF("Target reached");
        had_a_target_pos = false;
        k_event_set(&nav_events, NAV_EVENT_DESTINATION_REACHED);
        k_work_cancel_delayable(&recompute_path_work);
        k_work_cancel_delayable(&timeout_work);
        return;
    }
    k_work_reschedule_for_queue(&nav_workq, k_work_delayable_from_work(work), K_MSEC(1000 / 25));
}

static void recompute_path_work_handler(struct k_work *work)
{
    const int nb_try = 4;
    pos2_t robot_pos;
    int retry_counter = nb_try;
    while (retry_counter) {
        poklegscom_get_pos(&robot_pos);
        memcpy(in_use_astar_grid, astar_grids[current_astar_grid], sizeof(in_use_astar_grid));
        int clearance_size = nb_try - retry_counter;
        LOG_INF("Recompute path with clearance size %d", clearance_size);
        astar_grid_flood_fill_circle(in_use_astar_grid, BOARD_TO_ASTAR_GRID_X(robot_pos.x), BOARD_TO_ASTAR_GRID_Y(robot_pos.y), clearance_size, ASTAR_NODE_EMPTY);
        astar_grid_flood_fill_circle(in_use_astar_grid, BOARD_TO_ASTAR_GRID_X(target_pos.x), BOARD_TO_ASTAR_GRID_Y(target_pos.y), clearance_size, ASTAR_NODE_EMPTY);
        int err = go_to_with_pathfinding(in_use_astar_grid, &robot_pos, &target_pos);
        if (!err) {
            break;
        }
        retry_counter--;
    }
    k_work_reschedule_for_queue(&nav_workq, k_work_delayable_from_work(work), K_MSEC(1000));
}

static void timeout_work_handler(struct k_work *work)
{
    LOG_INF("Timeout");
    k_event_set(&nav_events, NAV_EVENT_TIMEOUT);
}

void obstacle_detection_event_handler(bool obstacle_detected) {
    if (obstacle_detected) {
        LOG_INF("Obstacle detected");
        poklegscom_set_break(1);
        if (current_mode == NAV_MODE_WITH_PATHFINDING) {
            k_work_reschedule_for_queue(&nav_workq, &recompute_path_work, K_NO_WAIT);
        }
    } else {
        LOG_INF("No obstacle detected");
        poklegscom_set_break(0);
    }
}

void update_obstacle_detection_state(bool obstacle_detected)
{
    if (current_obstacle_detected_state != obstacle_detected) {
        current_obstacle_detected_state = obstacle_detected;
        if (!had_a_target_pos) {
            return;
        }
        obstacle_detection_event_handler(obstacle_detected);
    }
}


static void add_rectangle_obstacle_to_grid(char *grid, point2_t point, float_t width, float height)
{
    int rx = BOARD_TO_ASTAR_GRID_X(point.x);
    int ry = BOARD_TO_ASTAR_GRID_Y(point.y);
    int astar_width = BOARD_TO_ASTAR_GRID_DIM(width);
    int astar_height = BOARD_TO_ASTAR_GRID_DIM(height);
    int margin = BOARD_TO_ASTAR_GRID_DIM(ROBOT_RADIUS + COLLISION_MARGIN);

    astar_grid_flood_fill_rectangle_with_margin(grid, rx, ry, astar_width, astar_height, margin, ASTAR_NODE_FULL);
}

static void add_circle_obstacle_to_grid(char *grid, point2_t point, float_t radius)
{
    int cx = BOARD_TO_ASTAR_GRID_X(point.x);
    int cy = BOARD_TO_ASTAR_GRID_Y(point.y);
    int r = BOARD_TO_ASTAR_GRID_DIM(ROBOT_RADIUS + COLLISION_MARGIN + radius);

    astar_grid_flood_fill_circle(grid, cx, cy, r, ASTAR_NODE_FULL);
}

static void add_lidar_point_obstacle_to_grid(char *grid, point2_t point)
{
    int cx = BOARD_TO_ASTAR_GRID_X(point.x);
    int cy = BOARD_TO_ASTAR_GRID_Y(point.y);
    int r = BOARD_TO_ASTAR_GRID_DIM(OBSTACLE_MARGIN);

    astar_grid_flood_fill_circle(grid, cx, cy, r, ASTAR_NODE_FULL);
}

static bool is_obstacled_too_close(float angle, float distance)
{
    if (distance > LIDAR_STOP_DISTANCE_MAX) {
        return false;
    }
    float ratio = MAX(distance - LIDAR_STOP_DISTANCE_MIN, 0.0f) /
                  (LIDAR_STOP_DISTANCE_MAX - LIDAR_STOP_DISTANCE_MIN);
    float test_angle = ratio * LIDAR_STOP_ANGLE_START + (1.0f - ratio) * LIDAR_STOP_ANGLE_END;
    return angle < test_angle / 2;
}

void lidar_callback(const struct lidar_point *points, size_t nb_points, void *user_data)
{
    float robot_dir;
    pos2_t robot_pos;
    poklegscom_get_dir(&robot_dir);
    poklegscom_get_pos(&robot_pos);

    int next_astar_grid = (current_astar_grid + 1) % 2;
    char *grid = astar_grids[current_astar_grid];
    memcpy(grid, static_astar_grid, sizeof(static_astar_grid));
    bool obstacle_detected = false;
    for (size_t i = 0; i < nb_points; i++) {
        const struct lidar_point *point = &points[i];

        if (point->distance < (ROBOT_CENTER_POLE_RADIUS + 1.0f)) {
            continue;
        }

        float point_dir_robot_ref = angle_normalize(point->angle);
        float point_dir_table_ref = angle_normalize(point_dir_robot_ref + robot_pos.a);
        float angle_dist_point_to_robot_dir =
            fabsf(angle_normalize(point_dir_table_ref - robot_dir));
        point2_t lidar_point2 = {
            .x = cosf(point_dir_table_ref) * point->distance + robot_pos.x,
            .y = sinf(point_dir_table_ref) * point->distance + robot_pos.y,
        };
        // LOG_INF("lidar_point2{.x=%.2f, .y=%.2f}", (double)lidar_point2.x,
        // (double)lidar_point2.y);
        const point2_t board_min = (point2_t){.x = BOARD_MIN_X, .y = BOARD_MIN_Y};
        const point2_t board_max = (point2_t){.x = BOARD_MAX_X, .y = BOARD_MAX_Y};
        if (!(POINT2_COMPARE(lidar_point2, >=, board_min) &&
              POINT2_COMPARE(lidar_point2, <=, board_max))) {
            continue;
        }
        // LOG_INF("robot_pos.a: %.2f", (double)RAD_TO_DEG(robot_pos.a));
        // LOG_INF("robot_dir: %.2f", (double)RAD_TO_DEG(robot_dir));
        // LOG_INF("point_dir_robot_ref: %.2f", (double)RAD_TO_DEG(point_dir_robot_ref));
        // LOG_INF("point_dir_table_ref: %.2f", (double)RAD_TO_DEG(point_dir_table_ref));
        // LOG_INF("angle_dist_point_to_robot_dir %.2f",
        // (double)RAD_TO_DEG(angle_dist_point_to_robot_dir)); LOG_INF("point->angle %.2f",
        // (double)RAD_TO_DEG(point->angle)); LOG_INF("point->distance %.2f",
        // (double)point->distance);

        if (is_obstacled_too_close(angle_dist_point_to_robot_dir, point->distance)) {
            obstacle_detected = true;
        }

        add_lidar_point_obstacle_to_grid(grid, lidar_point2);
    }
    current_astar_grid = next_astar_grid;
    update_obstacle_detection_state(obstacle_detected);
}

void nav_clear_obstacles(void) {
    memset(static_astar_grid, ASTAR_NODE_EMPTY, sizeof(static_astar_grid));
};

void nav_register_obstacle(struct nav_obstacle *obs) {
    switch (obs->type) {
        case NAV_OBSTACLE_TYPE_CIRCLE:
        add_circle_obstacle_to_grid(static_astar_grid, obs->data.circle.point, obs->data.circle.radius);
        break;
        case NAV_OBSTACLE_TYPE_RECTANGLE:
        {
            const struct nav_obstacle_rectangle *rec = &obs->data.rectangle;
            add_rectangle_obstacle_to_grid(static_astar_grid, rec->point, rec->width, rec->height);
        }
        break;
    }
};

int nav_init(void)
{
    static struct k_work_queue_config cfg = {
        .name = "nav_workq",
    };

    memset(astar_grids, ASTAR_NODE_EMPTY, sizeof(astar_grids));
    nav_clear_obstacles();

    poklegscom_set_break(1);
    k_work_queue_start(&nav_workq, nav_workq_stack, K_THREAD_STACK_SIZEOF(nav_workq_stack), 8,
                       &cfg);
    static struct lidar_callback lidar_clbk = {
        .clbk = lidar_callback,
    };
    lidar_register_callback(lidar_dev, &lidar_clbk);
    lidar_start(lidar_dev);
    return 0;
}

SYS_INIT(nav_init, APPLICATION, 0);
