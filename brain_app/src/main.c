#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>
#include "nav/nav.h"
#include "pokibot/lib/poktocol.h"
#include "pokibot/lib/pokutils.h"
#include "pokibot/poktypes.h"
#include "pomicontrol/pomicontrol.h"
#include "pokdefs.h"
#include "pokarm/pokarm.h"
#include "control/control.h"
#include <math.h>
#include <stdint.h>

LOG_MODULE_REGISTER(main);

#define STAND_DROP_DISTANCE_FROM_BORDER 0.075f

enum nav_obstacle_names {
    NAV_OBS_NAME_GRENIER,
    NAV_OBS_NAME_CRATES_TOP_RIGHT,
    NAV_OBS_NAME_LAST_STATIC_REGISTER,
};

// clang-format off
struct nav_obstacle static_obstacles[] = {
    // RAMPE
    [NAV_OBS_NAME_GRENIER] = {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = 0.0f,
                .y = 1775.0f,
            },
            .width = 1800.0f,
            .height = 450.0f,
        }
    },
    [NAV_OBS_NAME_CRATES_TOP_RIGHT] = {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MIN_X + 2825.0f,
                .y = 1200.0f,
            },
            .width = 150.0f,
            .height = 150.0f,
        }
    },
};
// clang-format on

void pomi_activate_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(pomi_activate_work, pomi_activate_work_handler);

void end_of_match_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(end_of_match_work, end_of_match_work_handler);

// DEFINE EVERYTHING FOR BLUE

const struct nav_obstacle start_zone = {
    .type = NAV_OBSTACLE_TYPE_RECTANGLE,
    .data.rectangle = {
        .point = {
            .x = BOARD_MIN_X + 2700.0f,
            .y = 1775.0f
        },
        .height = 450.0f,
        .width = 600.0f,
    }
};

const struct nav_obstacle end_zone = start_zone;

const point2_t start_point = {
    .x = BOARD_MIN_X + 2700.0f,
    .y = 1775.0f,
};

#define BOARD_END_POS_X (BOARD_MIN_X + 2800.0f)

const pos2_t end_pos = {
    .x = BOARD_END_POS_X,
    .y = 1775.0f,
    .a = 0.0f,
};;

#define TEMP_0_WIDTH   250.0f
#define CURSOR_WIDTH   100.0f

#define TEMP_0_X   (BOARD_MIN_X + 2875.0f)
#define TEMP_10_X   (BOARD_MIN_X + 2300.0f)

#define CURSOR_START_X (TEMP_0_X - TEMP_0_WIDTH / 2.0f + CURSOR_WIDTH / 2.0f)
#define CURSOR_END_X (TEMP_10_X - CURSOR_WIDTH / 3.0f)

pos2_t zone_exit_wp = {
    .x = BOARD_MIN_X + 2700.0f,
    .y = 1600.0f,
    .a = 0.0f,
};

pos2_t zone_enter_wp = {
    .x = BOARD_END_POS_X,
    .y = 1200.0f,
    .a = 0.0f,
};

pos2_t corridor_top_wp = {
    .x = BOARD_MIN_X + 2500.0f,
    .y = 1200.0f,
    .a = 0.0f,
};

pos2_t corridor_mid_wp = {
    .x = BOARD_MIN_X + 2500.0f,
    .y = 850.0f,
    .a = 0.0f,
};

pos2_t corridor_bottom_wp = {
    .x = BOARD_MIN_X + 2500.0f,
    .y = 300.0f,
    .a = 0.0f,
};

pos2_t pus_top_r_wp1 = {
    .x = BOARD_MAX_X - ROBOT_DIST_TO_FLAT_SIZE - 10.0f,
    .y = 850.0f,
    .a = 0.0f,
};

pos2_t pus_top_r_wp2 = {
    .x = BOARD_MAX_X - ROBOT_DIST_TO_FLAT_SIZE - 10.0f,
    .y = 1600.0f,
    .a = 0.0f,
};

pos2_t pus_top_r_go_bw_wp3 = {
    .x = BOARD_MAX_X - ROBOT_DIST_TO_FLAT_SIZE - 10.0f,
    .y = 1500.0f,
    .a = 0.0f,
};

pos2_t pus_top_r_recal = {
    .x = BOARD_MAX_X - ROBOT_DIST_TO_FLAT_SIZE - 50.0f,
    .y = 1500.0f,
    .a = 0.0f,
};

pos2_t cursor_wp1 = {
    .x = CURSOR_START_X,
    .y = ROBOT_RADIUS + 50.0f,
    .a = 0.0f,
};

pos2_t cursor_wp2 = {
    .x = CURSOR_START_X,
    .y = ROBOT_RADIUS + 10.0f,
    .a = 0.0f,
};

pos2_t cursor_wp3 = {
    .x = CURSOR_END_X,
    .y = ROBOT_RADIUS + 10.0f,
    .a = 0.0f,
};

pos2_t cursor_wp4 = {
    .x = CURSOR_END_X,
    .y = ROBOT_RADIUS + 50.0f,
    .a = 0.0f,
};

struct point2 convert_point_for_team(enum pokprotocol_team color, struct point2 point)
{
    if (color == POKTOCOL_TEAM_BLUE) {
        return point;
    }

    point.x = -point.x;
    return point;
}

struct pos2 convert_pos_for_team(enum pokprotocol_team color, struct pos2 pos, float offset)
{
    if (color == POKTOCOL_TEAM_BLUE) {
        pos.a += offset;
        return pos;
    }

    pos.x = -pos.x;
    pos.a = -(pos.a + M_PI) + offset;
    return pos;
}

struct pos2 convert_pos_for_team_no_angle(enum pokprotocol_team color, struct pos2 pos, float offset)
{
    if (color == POKTOCOL_TEAM_BLUE) {
        pos.a += offset;
        return pos;
    }

    pos.x = -pos.x;
    pos.a = pos.a + offset;
    return pos;
}

void pomi_activate_work_handler(struct k_work *work)
{
    LOG_INF("Activating pomis");
    pomicontrol_activate();
}

void end_of_match_work_handler(struct k_work *work)
{
    nav_cancel();
    LOG_INF("Match is over");
    k_sleep(K_FOREVER);
}

uint8_t score = 0;

#define WAIT_EVENTS_AND_HANDLE(events)                                                             \
    nav_wait_events(&events);                                                                      \
    if (events & NAV_EVENT_CANCELED) {                                                             \
        return -2;                                                                                 \
    } else if (events & NAV_EVENT_TIMEOUT) {                                                       \
        return -1;                                                                                 \
    }

enum recal_dir {
    recal_dir_n,
    recal_dir_s,
    recal_dir_e,
    recal_dir_w,
};

void wall_recalibration(enum pokprotocol_team color, pos2_t recal_start_pos, enum recal_dir dir)
{
    const float_t recal_dist = 200.0f; 

    uint32_t nav_events;
    nav_go_to_direct(convert_pos_for_team(color, recal_start_pos, 0.0f), K_SECONDS(10));
    nav_wait_events(&nav_events);
    
    LOG_INF(LOG_POS_ARGS("start_pos", recal_start_pos));

    pos2_t pos_recal = recal_start_pos;
    pos2_t pos_expected = recal_start_pos;
    
    float rotation_target;
    switch (dir) {
        case recal_dir_n:
            rotation_target = M_PI;
            pos_recal.y += recal_dist;
            pos_expected.y = BOARD_MAX_Y - ROBOT_CENTER_TO_RECAL_SIDE;
            break;
        case recal_dir_s:
            rotation_target = 0.0f;
            pos_recal.y -= recal_dist;
            pos_expected.y = BOARD_MIN_Y + ROBOT_CENTER_TO_RECAL_SIDE;
        break;
        case recal_dir_e:
            rotation_target = M_PI/2;
            pos_recal.x += recal_dist;
            pos_expected.x = BOARD_MAX_X - ROBOT_CENTER_TO_RECAL_SIDE;
        break;
        case recal_dir_w:
            rotation_target = -M_PI/2;
            pos_recal.x -= recal_dist;
            pos_expected.x = BOARD_MIN_X + ROBOT_CENTER_TO_RECAL_SIDE;
            break;
        default:
            return;
    }

    pos_recal.a = rotation_target;
    pos_expected.a = rotation_target;
    recal_start_pos.a = rotation_target;
    nav_go_to_direct(convert_pos_for_team(color, recal_start_pos, 0.0f), K_SECONDS(10));
    nav_wait_events(&nav_events);

    // LOG_INF("rotation_target %f", (double)rotation_target);
    // LOG_INF(LOG_POS_ARGS("recal_start_pos", recal_start_pos));
    // LOG_INF(LOG_POS_ARGS("pos_recal", pos_recal));
    // LOG_INF(LOG_POS_ARGS("pos_expected", pos_expected));

    pos_recal = convert_pos_for_team(color, pos_recal, 0.0f);
    pos_expected = convert_pos_for_team(color, pos_expected, 0.0f);
    recal_start_pos = convert_pos_for_team(color, recal_start_pos, 0.0f);

    // LOG_INF("Back to native color");
    // LOG_INF(LOG_POS_ARGS("recal_start_pos", recal_start_pos));
    // LOG_INF(LOG_POS_ARGS("pos_recal", pos_recal));
    // LOG_INF(LOG_POS_ARGS("pos_expected", pos_expected));

    nav_go_to_direct(pos_recal, K_SECONDS(10));
    nav_wait_events(&nav_events);
    nav_set_pos(pos_expected);

    nav_go_to_direct(recal_start_pos, K_SECONDS(10));
    nav_wait_events(&nav_events);
}


int match(enum pokprotocol_team color)
{
    uint32_t match_start_time_ms =  k_uptime_get_32();

    nav_set_pos(convert_pos_for_team_no_angle(color, CONVERT_POINT2_TO_POS2(start_point, 0.0f), M_PI));
    nav_set_brake(false);

    LOG_INF("Leaving start zone");
    uint32_t nav_events;
    nav_go_to_direct(convert_pos_for_team_no_angle(color, zone_exit_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, corridor_top_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("Left start zone");

    nav_go_to_direct(convert_pos_for_team_no_angle(color, corridor_mid_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, pus_top_r_wp1, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, pus_top_r_wp2, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    score += 1;
    pokuicom_send_score(score);

    LOG_INF("Back in start zone");
    nav_go_to_direct(convert_pos_for_team_no_angle(color, pus_top_r_go_bw_wp3, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);

    wall_recalibration(color, pus_top_r_recal, recal_dir_e);

    nav_go_to_direct(convert_pos_for_team_no_angle(color, corridor_top_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("Left start zone");

    nav_go_to_direct(convert_pos_for_team_no_angle(color, corridor_bottom_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);

    LOG_INF("CURSOR");
    nav_go_to_direct(convert_pos_for_team_no_angle(color, cursor_wp1, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, cursor_wp2, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, cursor_wp3, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, cursor_wp4, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);

    score += 1;
    pokuicom_send_score(score);

    LOG_INF("GO CORRIDOR");

    nav_go_to_direct(convert_pos_for_team_no_angle(color, corridor_bottom_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team_no_angle(color, corridor_top_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);


    nav_go_to_direct(convert_pos_for_team_no_angle(color, zone_enter_wp, M_PI), K_FOREVER);
    nav_wait_events(&nav_events);

    LOG_INF("GO HOME");

    // WAIT UNTIL 95s
    while ((k_uptime_get_32() - match_start_time_ms) < (95 * 1000)) {
        k_sleep(K_MSEC(100));
    }

    nav_go_to_direct(convert_pos_for_team_no_angle(color, end_pos, 2*M_PI), K_FOREVER);
    nav_wait_events(&nav_events);

    pokuicom_send_score(123);

    k_sleep(K_FOREVER);
    return 0;
}

void calibrate(enum pokprotocol_team color)
{
    nav_obstacle_detection(false);
    // nav_set_speed(200.0f, NAV_ANGULAR_VMAX_DEFAULT);

    k_sleep(K_MSEC(1000));

    nav_set_pos(convert_pos_for_team_no_angle(color, CONVERT_POINT2_TO_POS2(start_point, M_PI), 0.0f));
    nav_set_brake(false);

    k_sleep(K_SECONDS(1));

    wall_recalibration(color, CONVERT_POINT2_TO_POS2(start_point, M_PI), recal_dir_n);
    wall_recalibration(color, CONVERT_POINT2_TO_POS2(start_point, M_PI), recal_dir_e);

    // nav_set_speed(NAV_PLANAR_VMAX_DEFAULT, NAV_ANGULAR_VMAX_DEFAULT);
    nav_obstacle_detection(true);
}

int main(void)
{
    LOG_INF("Pokibot main start");

    for (int i = 0; i < NAV_OBS_NAME_LAST_STATIC_REGISTER; i++) {
        nav_register_obstacle(&static_obstacles[i]);
    }

    // pokarm_pinch(false);
    // pokarm_reset_pos();
    // k_sleep(K_MSEC(4000));
    // pokarm_pinch(true);
    // k_sleep(K_MSEC(1000));
    // pokarm_lift(true);

    LOG_INF("Regiter static obstacles: OK");

    // Wait for the starting cord to be put in to init few things
    while (pokuicom_get_tirette_status() != POKTOCOL_TIRETTE_STATUS_PLUGGED) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_TIRETTE_STATUS);
        k_sleep(K_MSEC(10));
    }

    enum pokprotocol_team color;
    while (pokuicom_get_team_color(&color)) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_TEAM);
        k_sleep(K_MSEC(10));
    }
    LOG_INF("Team: %s", color == POKTOCOL_TEAM_BLUE ? "blue" : "yellow");

    pokarm_reset_pos();
    pokarm_pinch(false);
    pokarm_deploy(false);
    score = 0;
    pokuicom_send_score(score);
    LOG_INF("Init of all the actuators: OK");

    LOG_INF("Calibration");
    calibrate(color);
    LOG_INF("Calibration: OK");

    while (pokuicom_get_tirette_status() != POKTOCOL_TIRETTE_STATUS_UNPLUGGED) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_TIRETTE_STATUS);
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Match started");
    k_work_schedule(&pomi_activate_work, K_SECONDS(85));
    k_work_schedule(&end_of_match_work, K_SECONDS(100));

    match(color);
    return 0;
}
