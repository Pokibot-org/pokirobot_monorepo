#include "pokibot/lib/poktocol.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pokibot/pokuicom.h>

LOG_MODULE_REGISTER(main);

int main(void)
{
    LOG_INF("Pokibot main start");
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
    k_sleep(K_FOREVER);
    return 0;
}
