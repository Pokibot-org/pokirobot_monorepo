#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>
#include "nav/nav.h"
#include "pokibot/poktypes.h"
#include "pomicontrol/pomicontrol.h"
#include "pokdefs.h"
#include "pokarm/pokarm.h"
#include "control/control.h"

LOG_MODULE_REGISTER(main);

#define STRAT_STAND_SCORE_LV1               4
#define STRAT_STAND_SCORE_LV2               8
#define STRAT_STAND_SCORE_LV3               16
#define STRAT_BANNER_SCORE                  20
#define STRAT_SHOWTIME_GROUPIE_SCORE        5
#define STRAT_SHOWTIME_SINGER_SCORE         5
#define STRAT_SHOWTIME_SINGER_ZONE_SCORE    10 // Depends on whant we can do
#define STRAT_SHOWTIME_EVERYONE_PARTY_SCORE 10
#define STRAT_END_GAME_IN_BACKSTAGE_SCORE   10

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

const pos2_t start_pos = {
    .x = BOARD_MIN_X + 2700.0f,
    .y = 1775.0f,
    .a = M_PI,
};

const pos2_t end_pos = start_pos;

#define TEMP_0_WIDTH   250.0f
#define CURSOR_WIDTH   100.0f

#define TEMP_0_X   (BOARD_MIN_X + 2875.0f)
#define TEMP_10_X   (BOARD_MIN_X + 2300.0f)

#define CURSOR_START_X (TEMP_0_X - TEMP_0_WIDTH / 2.0f + CURSOR_WIDTH / 2.0f)
#define CURSOR_END_X (TEMP_10_X - CURSOR_WIDTH / 3.0f)

pos2_t zone_exit_wp = {
    .x = BOARD_MIN_X + 2700.0f,
    .y = 1650.0f,
    .a = M_PI,
};

pos2_t corridor_top_wp = {
    .x = BOARD_MIN_X + 2500.0f,
    .y = 1300.0f,
    .a = M_PI,
};

pos2_t corridor_bottom_wp = {
    .x = BOARD_MIN_X + 2500.0f,
    .y = 300.0f,
    .a = M_PI,
};

pos2_t cursor_wp1 = {
    .x = CURSOR_START_X,
    .y = ROBOT_RADIUS + 50.0f,
    .a = M_PI,
};

pos2_t cursor_wp2 = {
    .x = CURSOR_START_X,
    .y = ROBOT_RADIUS + 10.0f,
    .a = M_PI,
};

pos2_t cursor_wp3 = {
    .x = CURSOR_END_X,
    .y = ROBOT_RADIUS + 10.0f,
    .a = M_PI,
};

pos2_t cursor_wp4 = {
    .x = CURSOR_END_X,
    .y = ROBOT_RADIUS + 50.0f,
    .a = M_PI,
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

#if CONFIG_POKISTRAT_MAIN
uint8_t score = 0;

#define WAIT_EVENTS_AND_HANDLE(events)                                                             \
    nav_wait_events(&events);                                                                      \
    if (events & NAV_EVENT_CANCELED) {                                                             \
        return -2;                                                                                 \
    } else if (events & NAV_EVENT_TIMEOUT) {                                                       \
        return -1;                                                                                 \
    }

int match(enum pokprotocol_team color)
{
    // uint32_t match_start_time_ms =  k_uptime_get_32();

    nav_set_pos(convert_pos_for_team(color, start_pos, 0.0f));
    nav_set_brake(false);

    LOG_INF("Leaving start zone");
    uint32_t nav_events;
    nav_go_to_direct(convert_pos_for_team(color, zone_exit_wp, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, corridor_top_wp, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("Left start zone");

    nav_go_to_direct(convert_pos_for_team(color, corridor_bottom_wp, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);

    LOG_INF("CURSOR");
    nav_go_to_direct(convert_pos_for_team(color, cursor_wp1, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, cursor_wp2, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, cursor_wp3, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, cursor_wp4, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);

    LOG_INF("GO CORRIDOR");

    nav_go_to_direct(convert_pos_for_team(color, corridor_bottom_wp, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, corridor_top_wp, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, zone_exit_wp, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);

    LOG_INF("GO HOME");

    nav_go_to_direct(convert_pos_for_team(color, end_pos, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);

    score += STRAT_END_GAME_IN_BACKSTAGE_SCORE;
    pokuicom_send_score(score);

    k_sleep(K_FOREVER);
    return 0;
}
#else
// FOR CERTIF
int match(enum pokprotocol_team color)
{
    return 0;
}
#endif


// void calibrate(enum pokprotocol_team color)
// {
//     nav_obstacle_detection(false);
//     nav_set_speed(200.0f, NAV_ANGULAR_VMAX_DEFAULT);

//     uint32_t nav_events;
//     point2_t converted_start_point = convert_point_for_team(color, start_zone.data.rectangle.point);
//     point2_t start_point = {
//         .x = converted_start_point.x - start_zone.data.rectangle.width / 2 + ROBOT_DIST_TO_FLAT_SIZE,
//         .y = converted_start_point.y - start_zone.data.rectangle.height / 2 + ROBOT_RADIUS,
//     };
//     pos2_t start_pos = CONVERT_POINT2_TO_POS2(start_point, 0.0f);
//     nav_set_pos(start_pos);
//     k_sleep(K_MSEC(500));

//     nav_go_to_direct(CONVERT_POINT2_TO_POS2(converted_start_point, 0.0f), K_FOREVER);
//     nav_wait_events(&nav_events);

//     nav_set_speed(NAV_PLANAR_VMAX_DEFAULT, NAV_ANGULAR_VMAX_DEFAULT);
//     nav_obstacle_detection(true);
// }

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

    // LOG_INF("Calibration");
    // calibrate(color);
    // LOG_INF("Calibration: OK");

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
