#include <pokibot/drivers/pokmac.h>
#include <pokibot/lib/poktocol.h>
#include <pokibot/lib/pokutils.h>
#include <pokibot/poklegscom.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(poklegscom);

static const struct device *pokmac_dev = DEVICE_DT_GET(DT_CHOSEN(pokibot_poklegscom));
uint8_t tx_buffer[512];

pos2_t current_pos = {0};
float current_dir = 0.0f;

static int encode_and_send(const struct poktocol_msg *msg, bool confirmed) {
    int size = poktocol_encode(msg, tx_buffer, sizeof(tx_buffer));
    if (size <= 0) {
        LOG_ERR("Encode error");
        return -1;
    }
    return pokmac_send(pokmac_dev, tx_buffer, size, confirmed);
}


int poklegscom_set_pos(const pos2_t *pos)
{
    current_pos = *pos;
    struct poktocol_msg msg = {
        .header = {
            .cmd = POKTOCOL_CMD_TYPE_WRITE,
            .type = POKTOCOL_DATA_TYPE_LEGS_POS,
        },
        .data.legs_pos = *pos
    };
    return encode_and_send(&msg, true);
}

int poklegscom_set_waypoints(const pos2_t *waypoints, size_t nb_waypoints) {
    struct poktocol_msg msg = {
        .header = {
            .cmd = POKTOCOL_CMD_TYPE_WRITE,
            .type = POKTOCOL_DATA_TYPE_LEGS_WAYPOINTS,
        },
        .data.waypoints = {
            .wps = (pos2_t *)waypoints,
            .nb_wps = nb_waypoints
        }
    };
    return encode_and_send(&msg, true);
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
    struct poktocol_msg msg = {
        .header = {
            .cmd = POKTOCOL_CMD_TYPE_WRITE,
            .type = POKTOCOL_DATA_TYPE_LEGS_BREAK,
        },
        .data.legs_break = state
    };
    return encode_and_send(&msg, true);
}

static void poklegscom_rx_payload(uint8_t *payload_data, size_t payload_size) {
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
        case POKTOCOL_DATA_TYPE_LEGS_NAV_DATA:
            {
                union poktocol_msg_data data;
                if (poktocol_decode_data(payload_data, payload_size, &data)) {
                    LOG_ERR("Err decoding nav data");
                    return;
                }
                current_pos = data.legs_nav_data.pos;
                current_dir = data.legs_nav_data.dir;
            }
            break;
        default:
            LOG_ERR("Unsupported data type %d", msg.type);
            break;
    }
}

int poklegscom_init(void)
{
    static struct pokmac_recv_callback clbk = {
        .cb = poklegscom_rx_payload
    };
    pokmac_register_recv_callback(pokmac_dev, &clbk);

    return 0;
}

SYS_INIT(poklegscom_init, APPLICATION, 1);
