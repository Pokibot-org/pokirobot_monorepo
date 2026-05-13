#include "control.h"
#include <pokibot/lib/poktocol.h>
#include <pokibot/msm.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(control);

pos2_t current_pos;
float current_dir;

#define ROOT_TOPIC "poklegscom/0/"

static int format_pos(char *buff, size_t buff_size, const pos2_t *pos) {
    return snprintf(buff, buff_size, "{\"x\": %f, \"y\": %f, \"a\": %f}", (double)pos->x, (double)pos->y, (double)pos->a);
}

void pos_clbk(char *data, int len, void *user_data) {
    sscanf(data, "%f %f %f", &current_pos.x, &current_pos.y, &current_pos.a);
}

void dir_clbk(char *data, int len, void *user_data) {
    sscanf(data, "%f", &current_dir);
}


int control_start(struct control *obj)
{
    if (!obj) {
        return -1;
    }
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

    obj->ready = true;
    LOG_INF("Control simulator started");
    return 0;
}

int control_set_pos(struct control *obj, pos2_t pos, bool force)
{
    if (!obj) {
        return -1;
    }
    
    static char topic_buff[128];
    format_pos(topic_buff, sizeof(topic_buff), &pos);
    int ret = msm_send(ROOT_TOPIC "set_pos", topic_buff);
    
    current_pos = pos;
    k_sleep(K_MSEC(50));
    
    return ret;
}

int control_get_pos(struct control *obj, pos2_t *pos)
{
    if (!obj || !pos) {
        return -1;
    }
    
    *pos = current_pos;
    return 0;
}

int control_get_dir(struct control *obj, float *dir)
{
    if (!obj || !dir) {
        return -1;
    }
    
    *dir = current_dir;
    return 0;
}

int control_set_brake(struct control *obj, bool brake)
{
    if (!obj) {
        return -1;
    }
    LOG_INF("control_set_brake %d", brake);
    
    static char topic_buff[128];
    snprintf(topic_buff, sizeof(topic_buff), "{\"state\": %s}", brake ? "true" : "false");
    int ret = msm_send(ROOT_TOPIC "set_brake", topic_buff);
    
    obj->brake = brake;
    return ret;
}

int control_set_waypoints(struct control *obj, const pos2_t *src, int n)
{
    if (!obj || !src || n <= 0) {
        return -1;
    }
    
    LOG_INF("control_set_waypoints");
    static char topic_buff[4096];
    int size = sizeof(topic_buff);
    int offset = 0;
    
    offset += snprintf(topic_buff, size, "[");
    for (int i = 0; i < n; i++) {
        const pos2_t *pos = &src[i];
        offset += snprintf(&topic_buff[offset], sizeof(topic_buff) - offset, 
                          "{\"x\":%.3f,\"y\":%.3f,\"a\":%.3f}", 
                          (double)pos->x, (double)pos->y, (double)pos->a);
        if (i < n - 1) {
            offset += snprintf(&topic_buff[offset], sizeof(topic_buff) - offset, ",");
        }
    }
    offset += snprintf(&topic_buff[offset], sizeof(topic_buff) - offset, "]");
    
    int ret = msm_send(ROOT_TOPIC "set_waypoints", topic_buff);
    
    if (ret == 0 && n <= CONTROL_WAYPOINTS_N) {
        obj->waypoints.n = n;
        for (int i = 0; i < n; i++) {
            obj->waypoints.wps[i] = src[i];
        }
    }
    
    return ret;
}