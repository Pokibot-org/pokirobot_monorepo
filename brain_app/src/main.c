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

int main(void)
{
    LOG_INF("Pokibot main start");

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
