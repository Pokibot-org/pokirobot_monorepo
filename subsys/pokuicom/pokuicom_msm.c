#include <pokibot/lib/poktocol.h>
#include <pokibot/pokuicom.h>
#include <pokibot/msm.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokuicom);

bool match_started = false;
bool has_color_info = false;
enum pokprotocol_team received_color;


#define ROOT_TOPIC "pokuicom/0/"


int pokuicom_request(enum poktocol_data_types type)
{
    static char topic_buff[128];
    snprintf(topic_buff, sizeof(topic_buff), "{\"value\": %d}", type);
    return msm_send( ROOT_TOPIC "request", topic_buff);
}

void pokuicom_send_score(uint8_t score)
{
    static char topic_buff[128];
    snprintf(topic_buff, sizeof(topic_buff), "{\"value\": %d}", score);
    msm_send(ROOT_TOPIC "score", topic_buff);
    return;
}

bool pokuicom_is_match_started(void)
{
    return match_started;
}

int pokuicom_get_team_color(enum pokprotocol_team *color)
{
    if (!has_color_info) {
        return -1;
    }
    *color = received_color;
    return 0;
}

void team_clbk(char *data, int len, void *user_data) {
    sscanf(data, "%d", (int*)&received_color);
    has_color_info = true;
}

void match_clbk(char *data, int len, void *user_data) {
    match_started = true;
}

int pokuicom_init(void)
{

    static struct msm_topic topic = {
        .name = ROOT_TOPIC "team",
        .clbk = team_clbk
    };
    msm_topic_register(&topic);


    static struct msm_topic topic_match = {
        .name = ROOT_TOPIC "match",
        .clbk = match_clbk
    };
    msm_topic_register(&topic_match);

    LOG_INF("init ok");
    return 0;
}

SYS_INIT(pokuicom_init, APPLICATION, 1);
