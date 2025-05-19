#include "pokibot/lib/poktocol.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>
#include "nav/nav.h"
#include "pokdefs.h"

LOG_MODULE_REGISTER(main);

enum team_color {
    TEAM_COLOR_NONE,
    TEAM_COLOR_BLUE,
    TEAM_COLOR_YELLOW,
};

#define STRAT_GRADIN_SCORE_LV1              4
#define STRAT_GRADIN_SCORE_LV2              8
#define STRAT_GRADIN_SCORE_LV3              16
#define STRAT_BANNER_SCORE                  20
#define STRAT_SHOWTIME_GROUPIE_SCORE        5
#define STRAT_SHOWTIME_SINGER_SCORE         5
#define STRAT_SHOWTIME_SINGER_ZONE_SCORE    10 // Depends on whant we can do
#define STRAT_SHOWTIME_EVERYONE_PARTY_SCORE 10
#define STRAT_END_GAME_IN_BACKSTAGE_SCORE   10

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

// DEFINE EVERYTHING FOR BLUE

const pos2_t start_pos = {
    .x = BOARD_MIN_X + 1.775f,
    .y = 0.225f,
    .a = 0.0f
};

struct point2 convert_point_for_team(enum team_color color, struct point2 point)
{
    if (color == TEAM_COLOR_BLUE) {
        return point;
    }

    point.x = -point.x;
    return point;
}

struct pos2 convert_pos_for_team(enum team_color color, struct pos2 pos)
{
    if (color == TEAM_COLOR_BLUE) {
        return pos;
    }

    pos.x = -pos.x;
    pos.a = pos.a + M_PI;
    if (pos.a > 2 * M_PI) {
        pos.a -= 2 * M_PI;
    }
    return pos;
}

#if CONFIG_POKISTRAT_MAIN
int match(enum pokprotocol_team color) {
    pos2_t start_pos = {.x = BOARD_MIN_X + 0.5f, .y = BOARD_MIN_Y + 0.5f, .a = 0.0f};
    nav_set_pos(&start_pos);

    nav_set_break(false);

    pos2_t waypoints[] = {
        {.x = BOARD_MIN_X + 0.5f, .y = BOARD_MIN_Y + 1.5f, .a = M_PI_2},
        {.x = BOARD_MIN_X + 2.5f, .y = BOARD_MIN_Y + 1.5f, .a = M_PI},
        {.x = BOARD_MIN_X + 2.5f, .y = BOARD_MIN_Y + 0.5f, .a = 3 * M_PI / 2},
        {.x = BOARD_MIN_X + 0.5f, .y = BOARD_MIN_Y + 0.5f, .a = 0},
    };

    uint32_t nav_events;
    nav_go_to(&waypoints[0], K_SECONDS(5));
    nav_wait_events(&nav_events);
    nav_go_to(&waypoints[1], K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to(&waypoints[2], K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to(&waypoints[3], K_FOREVER);
    nav_wait_events(&nav_events);

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
    nav_set_pos(&start_pos);
    nav_set_break(false);

    pos2_t waypoints[] = {
        {.x = start_pos.x, .y = 0.6f, .a = 0},
        {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = M_PI},
        {.x = BOARD_MIN_X + 2.225f, .y = 0.6f, .a = M_PI},
    };

    LOG_INF("Leaving start zone");
    uint32_t nav_events;
    nav_go_to_direct(&waypoints[0], K_FOREVER);
    nav_wait_events(&nav_events);
    LOG_INF("Left start zone");
    nav_go_to_direct(&waypoints[1], K_FOREVER);
    nav_wait_events(&nav_events);
    nav_go_to_direct(&waypoints[2], K_FOREVER);
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

    LOG_INF("Regiter static obstacles OK");

    enum pokprotocol_team color;
    while (pokuicom_get_team_color(&color)) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_TEAM);
        k_sleep(K_MSEC(10));
    }
    LOG_INF("Team: %s", color == POKTOCOL_TEAM_BLUE ? "blue" : "yellow");

    while (!pokuicom_is_match_started()) {
        pokuicom_request(POKTOCOL_DATA_TYPE_UI_MATCH_STARTED);
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Match started");
    match(color);
    return 0;
}
