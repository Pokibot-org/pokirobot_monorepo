#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pokibot/lib/poktocol.h"

static int pokprotocol_encode_dir(uint8_t *buffer, size_t buffer_size, const float *dir)
{
    const size_t size = sizeof(float);
    if (buffer_size < size) {
        return -1;
    }
    // Maybe encode to Q4.12
    memcpy(buffer, dir, size);
    return size;
}

static int pokprotocol_decode_dir(const uint8_t *buffer, float *dir)
{
    memcpy(dir, buffer, sizeof(float));
    return sizeof(float);
}

static int pokprotocol_encode_pos(uint8_t *buffer, size_t buffer_size, const pos2_t *pos)
{
    const size_t size = sizeof(pos2_t);
    if (buffer_size < size) {
        return -1;
    }
    // Maybe encode to Q4.12
    memcpy(buffer, pos, size);
    return size;
}

static int pokprotocol_decode_pos(const uint8_t *buffer, pos2_t *pos)
{
    memcpy(pos, buffer, sizeof(pos2_t));
    return sizeof(pos2_t);
}

int poktocol_encode(const struct poktocol_msg *msg, uint8_t *buffer, size_t buffer_size)
{
    if (buffer == NULL || msg == NULL || buffer_size < 8) {
        return -1; // Invalid parameters
    }

    // Encode the command and type
    size_t index = 0;
    buffer[index++] = msg->header.cmd;
    buffer[index++] = msg->header.type;

    if (msg->header.cmd != POKTOCOL_CMD_TYPE_WRITE) {
        return index;
    }

    // Encode the data based on the type
    switch (msg->header.type) {
        case POKTOCOL_DATA_TYPE_UI_SCORE:
            buffer[index++] = msg->data.score;
            break;
        case POKTOCOL_DATA_TYPE_UI_TEAM:
            buffer[index++] = msg->data.team;
            break;
        case POKTOCOL_DATA_TYPE_UI_MATCH_STATUS:
            buffer[index++] = msg->data.match_status;
            break;
        case POKTOCOL_DATA_TYPE_LEGS_POS: {
            int ret =
                pokprotocol_encode_pos(&buffer[index], buffer_size - index, &msg->data.legs_pos);
            if (ret < 0) {
                return -1;
            }
            index += ret;
        } break;
        case POKTOCOL_DATA_TYPE_LEGS_DIR: {
            int ret =
                pokprotocol_encode_dir(&buffer[index], buffer_size - index, &msg->data.legs_dir);
            if (ret < 0) {
                return -1;
            }
            index += ret;
        } break;
        case POKTOCOL_DATA_TYPE_LEGS_BREAK:
            buffer[index++] = msg->data.legs_break ? 1 : 0;
            break;
        case POKTOCOL_DATA_TYPE_LEGS_WAYPOINTS:
            buffer[index++] = msg->data.waypoints.nb_wps;
            for (size_t i = 0; i < msg->data.waypoints.nb_wps; i++) {
                int ret = pokprotocol_encode_pos(&buffer[index], buffer_size - index,
                                                 &msg->data.waypoints.wps[i]);
                if (ret < 0) {
                    return -1;
                }
                index += ret;
            }
            break;
        case POKTOCOL_DATA_TYPE_LEGS_NAV_DATA: {
            int ret;
            ret = pokprotocol_encode_pos(&buffer[index], buffer_size - index,
                                         &msg->data.legs_nav_data.pos);
            if (ret < 0) {
                return -1;
            }
            index += ret;
            ret = pokprotocol_encode_dir(&buffer[index], buffer_size - index,
                                         &msg->data.legs_nav_data.dir);
            if (ret < 0) {
                return -1;
            }
            index += ret;
        } break;
        default:
            return -1; // Unknown data type
    }

    return index; // Number of bytes written to the buffer
}

int poktocol_decode_header(const uint8_t *buffer, size_t buffer_size,
                           struct poktocol_header *header)
{
    if (buffer == NULL || header == NULL || buffer_size < 2) {
        return -1; // Invalid parameters
    }

    // Decode the command and type
    header->cmd = (enum poktocol_commands_types)buffer[0];
    header->type = (enum poktocol_data_types)buffer[1];
    return 0;
}

int poktocol_decode_data(const uint8_t *buffer, size_t buffer_size, union poktocol_msg_data *data)
{
    struct poktocol_header header;
    if (poktocol_decode_header(buffer, buffer_size, &header) < 0) {
        return -1;
    }

    int index = 2;
    // Decode the data based on the type
    switch (header.type) {
        case POKTOCOL_DATA_TYPE_UI_SCORE:
            if (buffer_size < 3) {
                return -1; // Not enough data for score
            }
            data->score = buffer[index++];
            break;
        case POKTOCOL_DATA_TYPE_UI_TEAM:
            if (buffer_size < 3) {
                return -1; // Not enough data for team
            }
            data->team = (enum pokprotocol_team)buffer[index++];
            break;
        case POKTOCOL_DATA_TYPE_UI_MATCH_STATUS:
            if (buffer_size < 3) {
                return -1; // Not enough data for team
            }
            data->match_status = (enum pokprotocol_match_status)buffer[index++];
            break;
        case POKTOCOL_DATA_TYPE_LEGS_POS:
            pokprotocol_decode_pos(&buffer[index++], &data->legs_pos);
            break;
        case POKTOCOL_DATA_TYPE_LEGS_DIR:
            pokprotocol_decode_dir(&buffer[index++], &data->legs_dir);
            break;
        case POKTOCOL_DATA_TYPE_LEGS_BREAK:
            data->legs_break = buffer[index++];
            break;
        case POKTOCOL_DATA_TYPE_LEGS_WAYPOINTS:
            if (data->waypoints.wps == NULL) {
                return -1;
            }
            data->waypoints.nb_wps = buffer[index++];
            for (size_t i = 0; i < data->waypoints.nb_wps; i++) {
                pokprotocol_decode_pos(&buffer[index], &data->waypoints.wps[i]);
            }
            break;
        case POKTOCOL_DATA_TYPE_LEGS_NAV_DATA: {
            index += pokprotocol_decode_pos(&buffer[index], &data->legs_nav_data.pos);
            index += pokprotocol_decode_dir(&buffer[index], &data->legs_nav_data.dir);
        } break;
        default:
            return -1; // Unknown data type
    }

    return 0; // Success
}
