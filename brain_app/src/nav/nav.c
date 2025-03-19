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
LOG_MODULE_REGISTER(nav);

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


#define ASTAR_NODE_EMPTY 1
#define ASTAR_NODE_FULL  0
#define GRID_SIZE_Y 21
#define GRID_SIZE_X 31
#define BOARD_BORDER_MARGIN  ROBOT_RADIUS
#define OBSTACLE_MARGIN      (ROBOT_RADIUS + OPPONENT_ROBOT_MAX_RADIUS)

#define BOARD_TO_ASTAR_GRID_X(x) (MIN(MAX((int)((x - BOARD_MIN_X - BOARD_BORDER_MARGIN) * (GRID_SIZE_X-1) / (BOARD_SIZE_X - 2*BOARD_BORDER_MARGIN)), 0), GRID_SIZE_X-1))
#define BOARD_TO_ASTAR_GRID_Y(y) (MIN(MAX((int)((y - BOARD_MIN_Y - BOARD_BORDER_MARGIN) * (GRID_SIZE_Y-1) / (BOARD_SIZE_Y - 2*BOARD_BORDER_MARGIN)), 0), GRID_SIZE_Y-1))
#define ASTAR_GRID_TO_BOARD_X(x) ((float)x * (BOARD_SIZE_X - 2*BOARD_BORDER_MARGIN) / (GRID_SIZE_X-1) + BOARD_MIN_X + BOARD_BORDER_MARGIN)
#define ASTAR_GRID_TO_BOARD_Y(y) ((float)y * (BOARD_SIZE_Y - 2*BOARD_BORDER_MARGIN) / (GRID_SIZE_Y-1) + BOARD_MIN_Y + BOARD_BORDER_MARGIN)

static char astar_grids[2][GRID_SIZE_Y * GRID_SIZE_X];
static int current_astar_grid = 0;

static const pos2_t sensivity = {
    .x = 0.01f,
    .y = 0.01f,
    .a = DEG_TO_RAD(2),
};

static void log_astar_grid(void) {
    char *current_grid = astar_grids[current_astar_grid];
    char line[GRID_SIZE_X * 2 + 1];
    memset(line, ' ', sizeof(line));
    line[GRID_SIZE_X] = '\0';
    LOG_INF("A* grid:");
    for (int y = 0; y < GRID_SIZE_Y; y++) {
        for (int x = 0; x < GRID_SIZE_X; x++) {
            int index = astar_getIndexByWidth(GRID_SIZE_X, x, GRID_SIZE_Y - y - 1);
            if (current_grid[index] == ASTAR_NODE_FULL) {
                line[x*2] = '1';
            } else {
                line[x*2] = '0';
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

static int go_to_with_pathfinding(const pos2_t *target_pos)
{
    uint32_t start_time = k_uptime_get_32();
    int sol_length;
    pos2_t robot_pos;
    poklegscom_get_pos(&robot_pos);
    LOG_INF("from{.x=%.2f, .y=%.2f}", (double)robot_pos.x, (double)robot_pos.y);
    LOG_INF("to{.x=%.2f, .y=%.2f}", (double)target_pos->x, (double)target_pos->y);
    int start = astar_getIndexByWidth(GRID_SIZE_X, BOARD_TO_ASTAR_GRID_X(robot_pos.x), BOARD_TO_ASTAR_GRID_Y(robot_pos.y));
    int end = astar_getIndexByWidth(GRID_SIZE_X, BOARD_TO_ASTAR_GRID_X(target_pos->x), BOARD_TO_ASTAR_GRID_Y(target_pos->y));
    LOG_INF("Indexes start %d | end %d", start, end);

    int *indexes =
        astar_compute((char *)astar_grids[current_astar_grid], &sol_length, GRID_SIZE_X, GRID_SIZE_Y, start, end);
    LOG_INF("Compute A* in %dms", k_uptime_get_32() - start_time);
    LOG_INF("Sol len %d | indexes %p", sol_length, (void *)indexes);
    if (!indexes) {
        LOG_ERR("Error in astar_compute");
        poklegscom_set_waypoints(target_pos, 1);
        log_astar_grid();
        return 0;
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
        return -1;
    }
    for (int i = 0; i < sol_length; i++) {
        int x, y;
        int node_index = sol_length - i - 1;
        astar_getCoordByWidth(GRID_SIZE_X, indexes[i], &x, &y);
        wps[node_index].a = robot_pos.a;
        wps[node_index].x = ASTAR_GRID_TO_BOARD_X(x);
        wps[node_index].y = ASTAR_GRID_TO_BOARD_Y(y);
    }
    k_free(indexes);
    wps[sol_length - 1] = *target_pos;
    poklegscom_set_waypoints(wps, sol_length);
    k_free(wps);
    LOG_INF("go_to_with_pathfinding in %dms", k_uptime_get_32() - start_time);
    return 0;
}

static void nav_stop_all(void) {
    struct k_work_sync sync;
    k_work_cancel_delayable_sync(&check_target_reached_work, &sync);
    k_work_cancel_delayable_sync(&recompute_path_work, &sync);
    k_work_cancel_delayable_sync(&timeout_work, &sync);
    k_event_clear(&nav_events, 0xFFFFFFFF);
}

int nav_go_to(const pos2_t *pos, k_timeout_t timeout)
{
    nav_stop_all();
    target_pos = *pos;
    had_a_target_pos = true;
    go_to_with_pathfinding(pos);
    k_work_schedule_for_queue(&nav_workq, &recompute_path_work, K_MSEC(1000));
    k_work_schedule_for_queue(&nav_workq, &check_target_reached_work, K_NO_WAIT);
    k_work_schedule_for_queue(&nav_workq, &timeout_work, timeout);
    return 0;
}

int nav_go_to_direct(const pos2_t *pos, k_timeout_t timeout)
{
    nav_stop_all();
    target_pos = *pos;
    had_a_target_pos = true;
    poklegscom_set_waypoints(pos, 1);
    k_work_schedule_for_queue(&nav_workq, &check_target_reached_work, K_NO_WAIT);
    k_work_schedule_for_queue(&nav_workq, &timeout_work, timeout);
    return 0;
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
    LOG_INF("Recompute path");
    go_to_with_pathfinding(&target_pos);
    k_work_reschedule_for_queue(&nav_workq, k_work_delayable_from_work(work), K_MSEC(1000));
}

static void timeout_work_handler(struct k_work *work)
{
    LOG_INF("Timeout");
    k_event_set(&nav_events, NAV_EVENT_TIMEOUT);
}

void update_obstacle_detection_state(bool obstacle_detected)
{
    if (current_obstacle_detected_state != obstacle_detected) {
        current_obstacle_detected_state = obstacle_detected;
        if (!had_a_target_pos) {
            return;
        }
        if (obstacle_detected) {
            LOG_INF("Obstacle detected");
            poklegscom_set_break(1);
        } else {
            LOG_INF("No obstacle detected");
            poklegscom_set_break(0);
        }
    }
}

static void add_obstacle_to_grid(char *grid, point2_t point) {
    int point_index = astar_getIndexByWidth(GRID_SIZE_X, BOARD_TO_ASTAR_GRID_X(point.x), BOARD_TO_ASTAR_GRID_Y(point.y));
    grid[point_index] = ASTAR_NODE_FULL;
    const int res = 32;
    for (int i=0; i < res; i++) {
        float a = (float)i * 2 * M_PI / res;
        int x = BOARD_TO_ASTAR_GRID_X(point.x + cosf(a) * OBSTACLE_MARGIN);
        int y = BOARD_TO_ASTAR_GRID_Y(point.y + sinf(a) * OBSTACLE_MARGIN);
        int grid_index = astar_getIndexByWidth(GRID_SIZE_X, x, y);
        grid[grid_index] = ASTAR_NODE_FULL;
    }
}

void lidar_callback(const struct lidar_point *points, size_t nb_points, void *user_data)
{
    float robot_dir;
    pos2_t robot_pos;
    poklegscom_get_dir(&robot_dir);
    poklegscom_get_pos(&robot_pos);

    int next_astar_grid = (current_astar_grid + 1) % 2;
    char *grid = astar_grids[current_astar_grid];
    memset(grid, ASTAR_NODE_EMPTY, sizeof(astar_grids[0]));
    bool obstacle_detected = false;
    for (size_t i = 0; i < nb_points; i++) {
        const struct lidar_point *point = &points[i];
        float point_dir_robot_ref = angle_normalize(-point->angle);
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
        bool obstacle_close = angle_dist_point_to_robot_dir < LIDAR_STOP_ANGLE / 2 &&
                              point->distance < LIDAR_STOP_DISTANCE;
        if (obstacle_close) {
            obstacle_detected = true;
        }

        add_obstacle_to_grid(grid, lidar_point2);
    }
    current_astar_grid = next_astar_grid;
    update_obstacle_detection_state(obstacle_detected);
}

int nav_init(void)
{
    static struct k_work_queue_config cfg = {
        .name = "nav_workq",
    };

    memset(astar_grids, ASTAR_NODE_EMPTY, sizeof(astar_grids));

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
