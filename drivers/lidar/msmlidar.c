#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/slist.h>
#include <pokibot/msm.h>
#include <pokibot/drivers/lidar.h>
#include <zephyr/data/json.h>

LOG_MODULE_REGISTER(msmlidar);

#define DT_DRV_COMPAT pokibot_msmlidar

#define MSM_TOPIC_ROOT "lidar/"

#define MAX_LIDAR_POINTS 360

struct msmlidar_config {
    int id;
};

struct msmlidar_data {
    char topic[128];
    char lidar_points_topic[128];
    bool started;
    sys_slist_t callbacks;
    struct lidar_point points[MAX_LIDAR_POINTS];
    size_t nb_points;
    struct k_work work_call_clbks;
};

struct json_lidar_point {
    struct json_obj_token angle;
    struct json_obj_token distance;
    int32_t intensity;
};

static const struct json_obj_descr json_lidar_point_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct json_lidar_point, angle, JSON_TOK_FLOAT),
    JSON_OBJ_DESCR_PRIM(struct json_lidar_point, distance, JSON_TOK_FLOAT),
    JSON_OBJ_DESCR_PRIM(struct json_lidar_point, intensity, JSON_TOK_NUMBER),
};

struct json_lidar_points {
    struct json_lidar_point points[MAX_LIDAR_POINTS];
    size_t nb_points;
};

struct json_obj_descr json_lidar_points_descr[] = {
    JSON_OBJ_DESCR_OBJ_ARRAY(struct json_lidar_points, points, MAX_LIDAR_POINTS, nb_points,
                             json_lidar_point_descr, ARRAY_SIZE(json_lidar_point_descr)),
};


void call_callbacks_work_handler(struct k_work *work) {
   	struct msmlidar_data *data = CONTAINER_OF(work, struct msmlidar_data, work_call_clbks);

    sys_snode_t *cur;
    SYS_SLIST_FOR_EACH_NODE(&data->callbacks, cur) {
        struct lidar_callback *cb = CONTAINER_OF(cur, struct lidar_callback, _node);
        cb->clbk(data->points, data->nb_points, cb->user_data);
    }
}

static char *buff_topic_builder(const struct device *dev, char *buffer, size_t buffer_len, char *ep)
{
    const struct msmlidar_config *cfg = dev->config;

    if (ep != NULL) {
        snprintf(buffer, buffer_len, MSM_TOPIC_ROOT "%d/%s", cfg->id, ep);
    } else {
        snprintf(buffer, buffer_len, MSM_TOPIC_ROOT "%d", cfg->id);
    }
    return buffer;
}

static char *topic_builder(const struct device *dev, char *ep)
{
    struct msmlidar_data *data = dev->data;
    return buff_topic_builder(dev, data->topic, sizeof(data->topic), ep);
}

/******************************************************************************
 * PUBLIC INTERFACE
 ******************************************************************************/

static int msmlidar_start(const struct device *dev)
{
    struct msmlidar_data *data = dev->data;
    data->started = true;
    return 0;
}

static int msmlidar_stop(const struct device *dev)
{
    struct msmlidar_data *data = dev->data;
    data->started = false;
    return 0;
}

static int msmlidar_register_callback(const struct device *dev, struct lidar_callback *clbk)
{
    struct msmlidar_data *data = dev->data;
    sys_slist_append(&data->callbacks, &clbk->_node);
    return 0;
}

static int decode_float(const struct json_obj_token *token, float *num)
{
    char *endptr;
    char prev_end;
    char *token_end = token->start + token->length;
    prev_end = *token_end;
    *token_end = '\0';

    errno = 0;
    *num = strtof(token->start, &endptr);

    *token_end = prev_end;

    if (errno != 0) {
        return -errno;
    }

    if (endptr != token_end) {
        return -EINVAL;
    }

    return 0;
}

void lidar_points_clbk(char *payload, int payload_len, void *user_data)
{
    struct device *dev = (struct device *)user_data;
    struct msmlidar_data *data = dev->data;
    int err;

    if (!data->started) {
        return;
    }

    struct json_lidar_points json_points;
    err = json_arr_parse(payload, payload_len, json_lidar_points_descr, &json_points);
    if (err < 0) {
        LOG_ERR("json parsing error %d\npayload:\n%s", err, payload);
        return;
    }
    for (int i = 0; i < json_points.nb_points; i++) {
        struct json_lidar_point *jp = &json_points.points[i];
        struct lidar_point *p = &data->points[i];
        if (decode_float(&jp->angle, &p->angle)) {
            LOG_ERR("json float parsing error");
            return;
        }
        if (decode_float(&jp->distance, &p->distance)) {
            LOG_ERR("json float parsing error");
            return;
        }
        p->intensity = jp->intensity;
    }

    data->nb_points = json_points.nb_points;
    k_work_submit(&data->work_call_clbks);
}

static int msmlidar_init(const struct device *dev)
{
    struct msmlidar_data *data = dev->data;
    static struct msm_topic lidar_points_topic = {
        .clbk = lidar_points_clbk,
    };
    lidar_points_topic.name = buff_topic_builder(dev, data->lidar_points_topic,
                                                 sizeof(data->lidar_points_topic), "lidar_points");
    lidar_points_topic.user_data = (void *)dev;
    msm_topic_register(&lidar_points_topic);

    sys_slist_init(&data->callbacks);

    k_work_init(&data->work_call_clbks, call_callbacks_work_handler);

    msm_send(topic_builder(dev, "status"), "connected");
    return 0;
}

static const struct lidar_driver_api msmlidar_api = {
    .register_callback = msmlidar_register_callback,
    .start = msmlidar_start,
    .stop = msmlidar_stop,
};

#define MSMLIDAR_DEFINE(inst)                                                                      \
    static struct msmlidar_data msmlidar_data_##inst = {                                           \
        .started = false,                                                                          \
    };                                                                                             \
                                                                                                   \
    static const struct msmlidar_config msmlidar_config_##inst = {.id = DT_INST_PROP(inst, id)};   \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, &msmlidar_init, NULL, &msmlidar_data_##inst,                       \
                          &msmlidar_config_##inst, POST_KERNEL, 90, &msmlidar_api);

DT_INST_FOREACH_STATUS_OKAY(MSMLIDAR_DEFINE)
