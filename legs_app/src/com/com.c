#include <pokibot/drivers/pokmac.h>
#include <pokibot/lib/poktocol.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "control/control.h"

LOG_MODULE_REGISTER(com);

static const struct device *pokmac_dev = DEVICE_DT_GET(DT_NODELABEL(pokmac_strat));
pos2_t wps_buffer[128];
uint8_t tx_buffer[64];
uint8_t rx_buffer[1024];
uint32_t rx_size = 0;

struct control *control_obj;

static struct k_work publish_work;
static struct k_work rx_work;
static struct k_timer publish_timer;
K_THREAD_STACK_DEFINE(com_wq_area, 1024);
struct k_work_q com_wq;

static void encode_and_send(const struct poktocol_msg *msg)
{
    int size = poktocol_encode(msg, tx_buffer, sizeof(tx_buffer));
    if (size <= 0) {
        LOG_ERR("Encode error");
        return;
    }
    pokmac_send(pokmac_dev, tx_buffer, size, false);
}

void publish_work_handler(struct k_work *work)
{
    struct poktocol_msg msg = {
        .header =
            {
                .cmd = POKTOCOL_CMD_TYPE_WRITE,
                .type = POKTOCOL_DATA_TYPE_LEGS_NAV_DATA,
            },
    };
    control_get_pos(control_obj, &msg.data.legs_nav_data.pos);
    control_get_dir(control_obj ,&msg.data.legs_nav_data.dir);
    msg.data.legs_nav_data.pos.x *= 0.001f;
    msg.data.legs_nav_data.pos.y *= 0.001f;
    encode_and_send(&msg);
}

static void rx_process_data(uint8_t *payload_data, size_t payload_size)
{
    LOG_DBG("rx data of size %d", payload_size);
    struct poktocol_header msg;
    if (poktocol_decode_header(payload_data, payload_size, &msg)) {
        LOG_ERR("Header decode error");
        return;
    }
    if (msg.cmd != POKTOCOL_CMD_TYPE_WRITE) {
        LOG_ERR("Only handling write");
        return;
    }

    switch (msg.type) {
        case POKTOCOL_DATA_TYPE_LEGS_POS: {
            union poktocol_msg_data data;
            if (poktocol_decode_data(payload_data, payload_size, &data)) {
                LOG_ERR("Err decoding pos");
                return;
            }
            data.legs_pos.x *= 1000;
            data.legs_pos.y *= 1000;
            control_set_pos(control_obj, data.legs_pos);
        } break;
        case POKTOCOL_DATA_TYPE_LEGS_WAYPOINTS: {
            union poktocol_msg_data data = {.waypoints.wps = wps_buffer};
            if (poktocol_decode_data(payload_data, payload_size, &data)) {
                LOG_ERR("Err decoding wps");
                return;
            }
            for (int i = 0; i < data.waypoints.nb_wps; i++) {
                data.waypoints.wps[i].x *= 1000;
                data.waypoints.wps[i].y *= 1000;
            }
            control_set_waypoints(control_obj, data.waypoints.wps, data.waypoints.nb_wps);
        } break;
        case POKTOCOL_DATA_TYPE_LEGS_BREAK: {
            union poktocol_msg_data data;
            if (poktocol_decode_data(payload_data, payload_size, &data)) {
                LOG_ERR("Err decoding break");
                return;
            }
            control_set_brake(control_obj, data.legs_break);
        } break;
        case POKTOCOL_DATA_TYPE_LEGS_VMAXS: {
            union poktocol_msg_data data;
            if (poktocol_decode_data(payload_data, payload_size, &data)) {
                LOG_ERR("Err decoding legs vmax");
                return;
            }
            control_set_planar_vmax(control_obj, data.nav_vmax.planar_vmax);
            control_set_angular_vmax(control_obj, data.nav_vmax.angular_vmax);
        } break;
        default:
            LOG_ERR("Unsupported data type %d", msg.type);
            break;
    }
}

void rx_work_handler(struct k_work *work)
{
    rx_process_data(rx_buffer, rx_size);
}

static void com_rx_payload(uint8_t *payload_data, size_t payload_size)
{
    if (payload_size > sizeof(rx_buffer)) {
        LOG_ERR("Too much data");
        return;
    }

    memcpy(rx_buffer, payload_data, payload_size);
    rx_size = payload_size;
    k_work_submit_to_queue(&com_wq, &rx_work);
}

void publish_timer_expiry(struct k_timer *timer)
{
    k_work_submit_to_queue(&com_wq, &publish_work);
}

int com_start(struct control *ctrl_obj)
{
    control_obj = ctrl_obj;
    k_work_queue_init(&com_wq);
    k_work_queue_start(&com_wq, com_wq_area, K_THREAD_STACK_SIZEOF(com_wq_area), 12, NULL);

    k_work_init(&publish_work, publish_work_handler);
    k_work_init(&rx_work, rx_work_handler);
    k_timer_init(&publish_timer, publish_timer_expiry, NULL);

    static struct pokmac_recv_callback clbk = {.cb = com_rx_payload};
    pokmac_register_recv_callback(pokmac_dev, &clbk);

    k_timer_start(&publish_timer, K_NO_WAIT, K_MSEC(MSEC_PER_SEC / 50));
    return 0;
}
