#include <pokibot/lib/poktocol.h>
#include <pokibot/lib/pokutils.h>
#include <pokibot/poklegscom.h>
#include <pokibot/msm.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(poklegscom, CONFIG_POKLEGSCOM_LOG_LEVEL);

#define ROOT_TOPIC "poklegscom/0/"

pos2_t current_pos;
float current_dir;

static int format_pos(char *buff, size_t buff_size, const pos2_t *pos) {
    return snprintf(buff, buff_size, "{\"x\": %f, \"y\": %f, \"a\": %f}", (double)pos->x, (double)pos->y, (double)pos->a);
}

int poklegscom_set_pos(const pos2_t *pos)
{
    static char topic_buff[128];
    format_pos(topic_buff, sizeof(topic_buff), pos);
    int ret = msm_send( ROOT_TOPIC "set_pos", topic_buff);
    while (!POS2_COMPARE(current_pos, ==, *pos)) {k_yield();}
    return ret;
}

int poklegscom_set_waypoints(const pos2_t *waypoints, size_t nb_waypoints) {
    static char topic_buff[4096];
    int size = sizeof(topic_buff);
    int offset = 0;
    offset += snprintf(topic_buff, size, "[");
    for (int i=0; i < nb_waypoints; i++) {
        const pos2_t *pos = &waypoints[i];
        offset += snprintf(&topic_buff[offset], sizeof(topic_buff) - offset, "{\"x\":%.3f,\"y\":%.3f,\"a\":%.3f},", (double)pos->x, (double)pos->y, (double)pos->a);
    }
    topic_buff[offset-1] = ']';
    return msm_send( ROOT_TOPIC "set_waypoints", topic_buff);
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
    static char topic_buff[128];
    snprintf(topic_buff, sizeof(topic_buff), "{\"state\": %s}", state ? "true": "false");
    msm_send(ROOT_TOPIC "set_break", topic_buff);
    return 0;
}


void pos_clbk(char *data, int len, void *user_data) {
    sscanf(data, "%f %f %f", &current_pos.x, &current_pos.y, &current_pos.a);
}

void dir_clbk(char *data, int len, void *user_data) {
    sscanf(data, "%f", &current_dir);
}

int poklegscom_init(void)
{

    static struct msm_topic topic = {
        .name = ROOT_TOPIC "pos",
        .clbk = pos_clbk
    };
    msm_topic_register(&topic);

    static struct msm_topic dir_topic = {
        .name = ROOT_TOPIC "dir",
        .clbk = dir_clbk
    };
    msm_topic_register(&dir_topic);

    LOG_INF("init ok");
    return 0;
}

SYS_INIT(poklegscom_init, APPLICATION, 1);
