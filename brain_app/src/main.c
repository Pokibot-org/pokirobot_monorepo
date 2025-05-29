#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>
#include "nav/nav.h"
#include "pokibot/poktypes.h"
#include "pomicontrol/pomicontrol.h"
#include "pokdefs.h"
#include "pokarm/pokarm.h"

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
    NAV_OBS_NAME_RAMPE1,
    NAV_OBS_NAME_RAMPE2,
    NAV_OBS_NAME_OBS_MIDDLE_LEFT,
    NAV_OBS_NAME_OBS_MIDDLE_RIGHT,
    NAV_OBS_NAME_OBS_RAMP_LEFT,
    NAV_OBS_NAME_OBS_RAMP_RIGHT,
    NAV_OBS_NAME_OBS_BOTOM_CENTER_LEFT,
    NAV_OBS_NAME_OBS_BOTOM_CENTER_RIGHT,
    NAV_OBS_NAME_OBS_BOTOM_CORNER_LEFT,
    NAV_OBS_NAME_OBS_BOTOM_CORNER_RIGHT,
    NAV_OBS_NAME_OBS_TOP_LEFT,
    NAV_OBS_NAME_OBS_TOP_RIGHT,
    NAV_OBS_NAME_LAST_STATIC_REGISTER,
};

// clang-format off
struct nav_obstacle static_obstacles[] = {
    // RAMPE
    [NAV_OBS_NAME_RAMPE1] = {
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
    [NAV_OBS_NAME_RAMPE2] = {
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
    [NAV_OBS_NAME_OBS_MIDDLE_LEFT] = {
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
    [NAV_OBS_NAME_OBS_MIDDLE_RIGHT] = {
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
    [NAV_OBS_NAME_OBS_RAMP_LEFT] = {
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
    [NAV_OBS_NAME_OBS_RAMP_RIGHT] = {
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
    [NAV_OBS_NAME_OBS_BOTOM_CENTER_LEFT] = {
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
    [NAV_OBS_NAME_OBS_BOTOM_CENTER_RIGHT] = {
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
    [NAV_OBS_NAME_OBS_BOTOM_CORNER_LEFT] = {
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
    [NAV_OBS_NAME_OBS_BOTOM_CORNER_RIGHT] = {
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
    [NAV_OBS_NAME_OBS_TOP_LEFT] = {
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
    [NAV_OBS_NAME_OBS_TOP_RIGHT] = {
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
// clang-format on

void pomi_activate_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(pomi_activate_work, pomi_activate_work_handler);

void end_of_match_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(end_of_match_work, end_of_match_work_handler);

// DEFINE EVERYTHING FOR BLUE

const struct nav_obstacle end_zone = {
    .type = NAV_OBSTACLE_TYPE_RECTANGLE,
    .data.rectangle = {
        .height = 0.45f,
        .width = 0.45f,
        .point = {
            .x = BOARD_MIN_X + 2.625f,
            .y = 1.775f
        }
    }
};

const struct nav_obstacle left_zone = {
    .type = NAV_OBSTACLE_TYPE_RECTANGLE,
    .data.rectangle = {
        .height = 0.45f,
        .width = 0.45f,
        .point = {
            .x = BOARD_MIN_X + 0.225f,
            .y = 0.875f
        }
    }
};

const struct nav_obstacle start_zone = {
    .type = NAV_OBSTACLE_TYPE_RECTANGLE,
    .data.rectangle = {
        .height = 0.45f,
        .width = 0.45f,
        .point = {
            .x = BOARD_MIN_X + 1.775f,
            .y = 0.225f
        }
    }
};

const struct nav_obstacle left_corner_tiny_zone = {
    .type = NAV_OBSTACLE_TYPE_RECTANGLE,
    .data.rectangle = {
        .height = 0.15f,
        .width = 0.45f,
        .point = {
            .x = BOARD_MIN_X + 0.225f,
            .y = 0.0775f
        }
    }
};

const struct nav_obstacle center_tiny_zone = {
    .type = NAV_OBSTACLE_TYPE_RECTANGLE,
    .data.rectangle = {
        .height = 0.15f,
        .width = 0.45f,
        .point = {
            .x = BOARD_MIN_X + 2.225f,
            .y = 0.0775f
        }
    }
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
}

#if CONFIG_POKISTRAT_MAIN
uint8_t score = 0;

int start_second_target(enum pokprotocol_team color)
{
    const float consigne_a = - ROBOT_ARM_ANGLE_OFFSET;
    uint32_t nav_events;

    LOG_INF("SECOND TARGET");
    const struct nav_obstacle_rectangle cor = static_obstacles[NAV_OBS_NAME_OBS_MIDDLE_RIGHT].data.rectangle;
    const pos2_t wp2_1 = {
        .x = cor.point.x + cor.width / 2 + ROBOT_RADIUS + 0.05f,
        .y = cor.point.y,
        .a = -M_PI/2
    };

    pos2_t wp2_2 = wp2_1;
    wp2_2.y += ROBOT_RADIUS + 0.20f;
    pos2_t wp2_3 = wp2_2;
    wp2_3.x = cor.point.x;
    pos2_t wp2_4 = CONVERT_POINT2_TO_POS2(cor.point, -M_PI/2);
    pos2_t wp2_5 = CONVERT_POINT2_TO_POS2(start_zone.data.rectangle.point, -M_PI/2);
    pos2_t wp2_6 = CONVERT_POINT2_TO_POS2(start_zone.data.rectangle.point, -M_PI/2);
    wp2_6.y = 0.6f;

    k_timeout_t per_task_timeout = K_FOREVER;

    nav_go_to_direct(convert_pos_for_team(color, wp2_1, consigne_a), per_task_timeout);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, wp2_2, consigne_a), per_task_timeout);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, wp2_3, consigne_a), per_task_timeout);
    nav_wait_events(&nav_events);
    pokarm_deploy(true);
    nav_set_speed(200.0f, NAV_ANGULAR_VMAX_DEFAULT);
    nav_go_to_direct(convert_pos_for_team(color, wp2_4, consigne_a), per_task_timeout);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, wp2_5, consigne_a), per_task_timeout);
    nav_wait_events(&nav_events);
    score += STRAT_STAND_SCORE_LV1;
    pokuicom_send_score(score);
    nav_go_to_direct(convert_pos_for_team(color, wp2_6, consigne_a), per_task_timeout);
    nav_wait_events(&nav_events);

    pokarm_deploy(false);
    nav_set_speed(400.0f, NAV_ANGULAR_VMAX_DEFAULT);
    LOG_INF("SECOND TARGET OK");
    return 0;
}

int match(enum pokprotocol_team color)
{
    uint32_t match_start_time_ms =  k_uptime_get_32();

    pos2_t start_pos = CONVERT_POINT2_TO_POS2(convert_point_for_team(color, start_zone.data.rectangle.point), 0.0f);
    nav_set_pos(start_pos);
    nav_set_break(false);

    const pos2_t wp1 = {.x = start_zone.data.rectangle.point.x, .y = 0.6f, .a = 0.0f };
    const pos2_t wp2 = {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = -M_PI/2 };
    const pos2_t wp3 = {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = -M_PI/2 };
    const pos2_t wp4 = {
        .x = BOARD_MIN_X + 2.225f,
        .y = STAND_DROP_DISTANCE_FROM_BORDER + ROBOT_CENTER_TO_GRIPPER_DIST,
        .a = -M_PI/2
    };

    const float consigne_a = - ROBOT_ARM_ANGLE_OFFSET;

    LOG_INF("Leaving start zone");
    uint32_t nav_events;
    nav_go_to_direct(convert_pos_for_team_no_angle(color, wp1, 0.0f), K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("Left start zone");


    LOG_INF("FIRST TARGET");
    nav_go_to_direct(convert_pos_for_team(color, wp2, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(convert_pos_for_team(color, wp3, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);

    pokarm_deploy(true);
    nav_set_speed(200.0f, NAV_ANGULAR_VMAX_DEFAULT);

    nav_go_to_direct(convert_pos_for_team(color, wp4, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);

    score += STRAT_STAND_SCORE_LV1;
    pokuicom_send_score(score);

    nav_go_to_direct(convert_pos_for_team(color, wp3, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("FIRST TARGET OK");

    pokarm_deploy(false);
    nav_set_speed(400.0f, NAV_ANGULAR_VMAX_DEFAULT);

    start_second_target(color);

    LOG_INF("GO BACKSTAGE");
    const pos2_t end_pos = CONVERT_POINT2_TO_POS2(end_zone.data.rectangle.point, -M_PI/2);
    const pos2_t wpbs_0 = {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = -M_PI/2 };
    pos2_t wp0_wait_before_backstage = {
        .a = end_pos.a,
        .x = BOARD_MIN_X + 2.35f,
        .y = 1.0f
    };

    pos2_t wp1_wait_before_backstage = {
        .a = end_pos.a,
        .x = end_pos.x + 0.05f,
        .y = end_pos.y - ROBOT_RADIUS - end_zone.data.rectangle.height / 2 - 0.05f
    };

    nav_go_to_direct(convert_pos_for_team(color, wpbs_0, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);

    nav_go_to_direct(convert_pos_for_team(color, wp0_wait_before_backstage, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);

    nav_go_to_direct(convert_pos_for_team(color, wp1_wait_before_backstage, consigne_a), K_FOREVER);
    nav_wait_events(&nav_events);

    // WAIT UNTIL 95s
    while ((k_uptime_get_32() - match_start_time_ms) < (95 * 1000)) {
        k_sleep(K_MSEC(100));
    }

    nav_go_to_direct(convert_pos_for_team(color, end_pos, consigne_a), K_FOREVER);
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

void test_pomicontrol(void) {
    while (1) {
        pomicontrol_activate();
        k_sleep(K_MSEC(1000));
    }
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

    pokarm_reset_pos();
    pokarm_pinch(false);
    pokarm_deploy(false);
    score = 0;
    pokuicom_send_score(score);

    LOG_INF("Init of all the actuators: OK");

    enum pokprotocol_team color;
    while (pokuicom_get_team_color(&color)) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_TEAM);
        k_sleep(K_MSEC(10));
    }
    LOG_INF("Team: %s", color == POKTOCOL_TEAM_BLUE ? "blue" : "yellow");

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
