#include <pokibot/drivers/pokmac.h>
#include <pokibot/lib/poktocol.h>
#include <pokibot/lib/pokutils.h>
#include <pokibot/poklegscom.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(poklegscom);

static const struct device *pokmac_dev = DEVICE_DT_GET(DT_CHOSEN(pokibot_poklegs));

pos2_t current_pos;
float current_dir;

int poklegscom_set_pos(const pos2_t *pos)
{
    current_pos = *pos;
    // while (!POS2_COMPARE(current_pos, ==, *pos)) {k_yield();}
    return 0;
}

int poklegscom_set_waypoints(const pos2_t *waypoints, size_t nb_waypoints) {
    return 0;
};

int poklegscom_get_pos(pos2_t *pos) {
    *pos = current_pos;
    return 0;
}

int poklegscom_get_dir(float *dir) {
    *dir = current_dir;
    return 0;
}

int poklegscom_set_break(bool state)
{
    return 0;
}

int poklegscom_init(void)
{
    return 0;
}

SYS_INIT(poklegscom_init, APPLICATION, 1);
