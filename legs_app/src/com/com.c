#include <pokibot/drivers/pokmac.h>
#include <pokibot/lib/poktocol.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "control/control.h"

LOG_MODULE_REGISTER(com);

static const struct device *pokmac_dev = DEVICE_DT_GET(DT_NODELABEL(pokmac_strat));
pos2_t wps_buffer[128];
uint8_t tx_buffer[64];

static struct k_work publish_work;
static struct k_timer publish_timer;

static void encode_and_send(const struct poktocol_msg *msg) {
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
        .header = {
            .cmd = POKTOCOL_CMD_TYPE_WRITE,
            .type = POKTOCOL_DATA_TYPE_LEGS_NAV_DATA,
        },
    };
    control_get_pos(&msg.data.legs_nav_data.pos);
    control_get_dir(&msg.data.legs_nav_data.dir);
    encode_and_send(&msg);
}


static void com_rx_payload(uint8_t *payload_data, size_t payload_size) {
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
        case POKTOCOL_DATA_TYPE_LEGS_POS:
            {
                union poktocol_msg_data data;
                if (poktocol_decode_data(payload_data, payload_size, &data)) {
                    LOG_ERR("Err decoding pos");
                    return;
                }
                control_set_pos(data.legs_pos);
            }
            break;
        case POKTOCOL_DATA_TYPE_LEGS_WAYPOINTS:
            {
                union poktocol_msg_data data = {
                    .waypoints.wps = wps_buffer
                };
                if (poktocol_decode_data(payload_data, payload_size, &data)) {
                    LOG_ERR("Err decoding wps");
                    return;
                }
                control_set_waypoints(data.waypoints.wps, data.waypoints.nb_wps);
            }
            break;
        case POKTOCOL_DATA_TYPE_LEGS_BREAK:
            {
                union poktocol_msg_data data;
                if (poktocol_decode_data(payload_data, payload_size, &data)) {
                    LOG_ERR("Err decoding break");
                    return;
                }
                control_set_brake(data.legs_break);
            }
            break;
        default:
            LOG_ERR("Unsupported data type %d", msg.type);
            break;
    }
}


void publish_timer_expiry(struct k_timer *timer)
{
    k_work_submit(&publish_work);
}

int com_start(void)
{
    k_work_init(&publish_work, publish_work_handler);
    k_timer_init(&publish_timer, publish_timer_expiry, NULL);

    static struct pokmac_recv_callback clbk = {
        .cb = com_rx_payload
    };
    pokmac_register_recv_callback(pokmac_dev, &clbk);

    k_timer_start(&publish_timer, K_NO_WAIT, K_MSEC(MSEC_PER_SEC/100));
    return 0;
}
