#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "pokibot/lib/poktocol.h"

int poktocol_encode(const struct poktocol_msg *msg, uint8_t *buffer, size_t buffer_size) {
    if (buffer == NULL || msg == NULL || buffer_size < 8) {
        return -1; // Invalid parameters
    }

    // Encode the command and type
    size_t index = 0;
    buffer[index++] = msg->cmd;
    buffer[index++] = msg->type;

    // Encode the data based on the type
    switch (msg->type) {
        case POKTOCOL_DATA_TYPE_SCORE:
            buffer[index++] = msg->data.score;
            break;
        case POKTOCOL_DATA_TYPE_TEAM:
            buffer[index++] = msg->data.team;
            break;
        case POKTOCOL_DATA_TYPE_MATCH_STARTED:
            // No additional data for this type
            break;
        default:
            return -1; // Unknown data type
    }

    return index; // Number of bytes written to the buffer
}

int poktocol_decode(const uint8_t *buffer, size_t buffer_size, struct poktocol_msg *msg) {
    if (buffer == NULL || msg == NULL || buffer_size < 2) {
        return -1; // Invalid parameters
    }

    // Decode the command and type
    msg->cmd = (enum poktocol_commands_types)buffer[0];
    msg->type = (enum poktocol_data_types)buffer[1];

    // Decode the data based on the type
    switch (msg->type) {
        case POKTOCOL_DATA_TYPE_SCORE:
            if (buffer_size < 3) {
                return -1; // Not enough data for score
            }
            msg->data.score = buffer[2];
            break;
        case POKTOCOL_DATA_TYPE_TEAM:
            if (buffer_size < 3) {
                return -1; // Not enough data for team
            }
            msg->data.team = (enum pokprotocol_team)buffer[2];
            break;
        case POKTOCOL_DATA_TYPE_MATCH_STARTED:
            // No additional data for this type
            break;
        default:
            return -1; // Unknown data type
    }

    return 0; // Success
}
