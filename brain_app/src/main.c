#include "pokarm/pokarm.h"
#include "pokibot/lib/poktocol.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>
#include "nav/nav.h"
#include "pokibot/poktypes.h"
#include "pomicontrol/pomicontrol.h"
#include "pokdefs.h"

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

#define STAND_DROP_DISTANCE_FROM_BORDER     0.075f

struct nav_obstacle static_obstacles[] = {
    // RAMPE
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = 0.0f,
                .y = 1.90f,
            },
            .width = 1.7f,
            .height = 0.2f,
        }
    },
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = 0.0f,
                .y = 1.65f,
            },
            .width = 0.9f,
            .height = 0.3f,
        }
    },
    // MIDDLE OBSTACLES
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = -0.4f,
                .y = 0.95f,
            },
            .width = 0.4f,
            .height = 0.1f,
        },
    },
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = 0.4f,
                .y = 0.95f,
            },
            .width = 0.4f,
            .height = 0.1f,
        }
    },
    // TOP CENTER
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MIN_X + 0.825f,
                .y = 1.725f,
            },
            .width = 0.4f,
            .height = 0.1f,
        }
    },
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MAX_X - 0.825f,
                .y = 1.725f,
            },
            .width = 0.4f,
            .height = 0.1f,
        }
    },
    // BOTTOM CENTER
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MIN_X + 0.775f,
                .y = 0.25f,
            },
            .width = 0.4f,
            .height = 0.1f,
        }
    },
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MAX_X - 0.775f,
                .y = 0.25f,
            },
            .width = 0.4f,
            .height = 0.1f,
        }
    },
    // BOTTOM LR
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MIN_X + 0.075f,
                .y = 0.4f,
            },
            .width = 0.1f,
            .height = 0.4f,
        }
    },
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MAX_X - 0.075f,
                .y = 0.4f,
            },
            .width = 0.1f,
            .height = 0.4f,
        }
    },
    // TOP LR
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MIN_X + 0.075f,
                .y = 1.325f,
            },
            .width = 0.1f,
            .height = 0.4f,
        }
    },
    (struct nav_obstacle) {
        .type = NAV_OBSTACLE_TYPE_RECTANGLE,
        .data.rectangle = {
            .point = {
                .x = BOARD_MAX_X - 0.075f,
                .y = 1.325f,
            },
            .width = 0.1f,
            .height = 0.4f,
        }
    },
};

void pomi_activate_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(pomi_activate_work, pomi_activate_work_handler);

// DEFINE EVERYTHING FOR BLUE

const pos2_t start_pos = {
    .x = BOARD_MIN_X + 1.775f,
    .y = 0.225f,
    .a = 0.0f
};

struct point2 convert_point_for_team(enum pokprotocol_team color, struct point2 point)
{
    if (color == POKTOCOL_TEAM_BLUE) {
        return point;
    }

    point.x = -point.x;
    return point;
}

struct pos2 convert_pos_for_team(enum pokprotocol_team color, struct pos2 pos)
{
    if (color == POKTOCOL_TEAM_BLUE) {
        return pos;
    }

    pos.x = -pos.x;
    pos.a = pos.a + M_PI;
    if (pos.a > 2 * M_PI) {
        pos.a -= 2 * M_PI;
    }
    return pos;
}

int strat_set_break(bool status) {
    return nav_set_break(status);
}

int strat_set_pos(enum pokprotocol_team color, const pos2_t *pos) {
    const pos2_t converted_pos = convert_pos_for_team(color, *pos);
    return nav_set_pos(&converted_pos);
}

int strat_go_to(enum pokprotocol_team color, const pos2_t *pos, k_timeout_t timeout) {
    const pos2_t converted_pos = convert_pos_for_team(color, *pos);
    return nav_go_to(&converted_pos, timeout);
}

int strat_go_to_direct(enum pokprotocol_team color, const pos2_t *pos, k_timeout_t timeout){
    const pos2_t converted_pos = convert_pos_for_team(color, *pos);
    return nav_go_to_direct(&converted_pos, timeout);
}

void pomi_activate_work_handler(struct k_work *work)
{
    LOG_INF("Activating pomis");
    pomicontrol_activate();
}

#if CONFIG_POKISTRAT_MAIN
int match(enum pokprotocol_team color) {
    k_sleep(K_FOREVER);
    return 0;
}
#else
// FOR CERTIF
int match(enum pokprotocol_team color) {
    const pos2_t start_pos = {
        .x = BOARD_MIN_X + 1.775f,
        .y = 0.225f,
        .a = 0.0f
    };
    strat_set_pos(color, &start_pos);
    strat_set_break(false);

    const pos2_t wp1 = {.x = start_pos.x, .y = 0.6f, .a = 0};
    const pos2_t wp2 = {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = -M_PI/2 - ROBOT_ARM_ANGLE_OFFSET};
    const pos2_t wp3 = {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = -M_PI/2 - ROBOT_ARM_ANGLE_OFFSET};
    const pos2_t wp4 = {.x = BOARD_MIN_X + 2.225f, .y = STAND_DROP_DISTANCE_FROM_BORDER + ROBOT_CENTER_TO_GRIPPER_DIST, .a = -M_PI/2 - ROBOT_ARM_ANGLE_OFFSET};

    const pos2_t end_pos = {
        .x = BOARD_MIN_X + 2.655f,
        .y = 1.775f,
        .a = -M_PI/2 - ROBOT_ARM_ANGLE_OFFSET
    };

    LOG_INF("Leaving start zone");
    uint32_t nav_events;
    strat_go_to_direct(color, &wp1, K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("Left start zone");
    strat_go_to_direct(color, &wp2, K_FOREVER);
    nav_wait_events(&nav_events);
    strat_go_to_direct(color, &wp3, K_FOREVER);
    nav_wait_events(&nav_events);

    pokarm_deploy(true);

    strat_go_to_direct(color, &wp4, K_FOREVER);
    nav_wait_events(&nav_events);
    strat_go_to_direct(color, &wp3, K_FOREVER);
    nav_wait_events(&nav_events);

    strat_go_to(color, &end_pos, K_FOREVER);
    nav_wait_events(&nav_events);

    k_sleep(K_FOREVER);
    return 0;
}
#endif

int main(void)
{
    LOG_INF("Pokibot main start");

    for (int i = 0; i < ARRAY_SIZE(static_obstacles); i++) {
        nav_register_obstacle(&static_obstacles[i]);
    }

    LOG_INF("Regiter static obstacles: OK");

    // Wait for the starting cord to be put in to init few things
    while (pokuicom_get_match_status() != POKUICOM_MATCH_STATUS_SETUP) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_MATCH_STATUS);
        k_sleep(K_MSEC(10));
    }

    pokarm_pinch(false);
    pokarm_deploy(false);
    pokarm_reset_pos();

    LOG_INF("Init of all the actuatos: OK");

    enum pokprotocol_team color;
    while (pokuicom_get_team_color(&color)) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_TEAM);
        k_sleep(K_MSEC(10));
    }
    LOG_INF("Team: %s", color == POKTOCOL_TEAM_BLUE ? "blue" : "yellow");

    while (pokuicom_get_match_status() != POKUICOM_MATCH_STATUS_STARTED) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_MATCH_STATUS);
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Match started");
    k_work_schedule(&pomi_activate_work, K_SECONDS(85));

    match(color);
    return 0;
}
