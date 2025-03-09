#include "pokibot/lib/poktocol.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>
#include <pokibot/poklegscom.h>

LOG_MODULE_REGISTER(main);

int main(void)
{
    LOG_INF("Pokibot main start");


    poklegscom_set_break(1);
    enum pokprotocol_team color;
    while (pokuicom_get_team_color(&color)) {
        pokuicom_request(POKTOCOL_DATA_TYPE_TEAM);
        k_sleep(K_MSEC(10));
    }
    LOG_INF("Team: %s", color == POKTOCOL_TEAM_BLUE ? "blue" : "yellow");

    while (!pokuicom_is_match_started()) {
        pokuicom_request(POKTOCOL_DATA_TYPE_MATCH_STARTED);
        k_sleep(K_MSEC(10));
    }

    LOG_INF("Match started");

    pos2_t start_pos = {.x = 0.5, .y=0.5, .a=0};
    poklegscom_set_pos(&start_pos);
    poklegscom_set_break(0);

    pos2_t waypoints[] = {
        {.x = 0.5, .y=1.5, .a=0},
        {.x = 2.5, .y=1.5, .a=0},
        {.x = 2.5, .y=0.5, .a=0},
        {.x = 0.5, .y=0.5, .a=0},
    };
    poklegscom_set_waypoints(waypoints, sizeof(waypoints));

    k_sleep(K_FOREVER);
    return 0;
}
