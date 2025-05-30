#include "pokibot/drivers/pokmac.h"
#include "pokibot/lib/poktocol.h"
#include "pokibot/pokuicom.h"
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokuicom, CONFIG_POKUICOM_LOG_LEVEL);

static const struct device *pokmac_dev = DEVICE_DT_GET(DT_CHOSEN(pokibot_pokuicom));

enum pokprotocol_tirette_status tirette_status = POKTOCOL_TIRETTE_STATUS_UNKNOWN;
bool has_color_info = false;
enum pokprotocol_team received_color;

static void pokuicom_receive(struct poktocol_msg *msg)
{
    LOG_DBG("Received msg type %d", msg->header.type);

    switch (msg->header.type) {
        case POKTOCOL_DATA_TYPE_UI_SCORE:
            break;
        case POKTOCOL_DATA_TYPE_UI_TEAM:
            has_color_info = true;
            received_color = msg->data.team;
            break;
        case POKTOCOL_DATA_TYPE_UI_TIRETTE_STATUS:
            tirette_status = msg->data.tirette_status;
            break;
        default:
            break;
    }
}

static int pokuicom_send(const struct poktocol_msg *msg)
{
    uint8_t buffer[16];
    size_t encoded_size = poktocol_encode(msg, buffer, sizeof(buffer));
    if (encoded_size < 0) {
        LOG_ERR("Encode error ui type: %d", msg->header.type);
        return -1;
    }
    return pokmac_send(pokmac_dev, buffer, encoded_size, false);
}

int pokuicom_request(enum poktocol_data_types type)
{
    LOG_DBG("Sending request of type %d", type);
    struct poktocol_msg msg = {.header = {.cmd = POKTOCOL_CMD_TYPE_REQUEST, .type = type}};
    return pokuicom_send(&msg);
}

void pokuicom_send_score(uint8_t score)
{
    struct poktocol_msg msg = {
        .header = {.cmd = POKTOCOL_CMD_TYPE_WRITE, .type = POKTOCOL_DATA_TYPE_UI_SCORE},
        .data.score = score};

    pokuicom_send(&msg);
}

enum pokprotocol_tirette_status pokuicom_get_tirette_status(void)
{
    return tirette_status;
}

int pokuicom_get_team_color(enum pokprotocol_team *color)
{
    if (!has_color_info) {
        return -1;
    }
    *color = received_color;
    return 0;
}

void rx_cv(uint8_t *payload_data, size_t payload_size)
{
    static struct poktocol_msg msg;
    if (poktocol_decode_header(payload_data, payload_size, &msg.header) < 0) {
        LOG_ERR("poktocol_decode error");
        return;
    }
    if (poktocol_decode_data(payload_data, payload_size, &msg.data) < 0) {
        LOG_ERR("poktocol_decode error");
        return;
    }
    pokuicom_receive(&msg);
}

int pokuicom_init(void)
{
    static struct pokmac_recv_callback clbk = {.cb = rx_cv};
    pokmac_register_recv_callback(pokmac_dev, &clbk);
    LOG_INF("init ok");
    return 0;
}

SYS_INIT(pokuicom_init, APPLICATION, 1);
