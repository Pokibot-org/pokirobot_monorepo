#ifndef POKTOCOL_H
#define POKTOCOL_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

enum poktocol_commands_types {
    POKTOCOL_CMD_TYPE_WRITE,
    POKTOCOL_CMD_TYPE_REQUEST,
};

enum poktocol_data_types {
    POKTOCOL_DATA_TYPE_SCORE,
    POKTOCOL_DATA_TYPE_TEAM,
    POKTOCOL_DATA_TYPE_MATCH_STARTED,
};

enum pokprotocol_team {
    POKTOCOL_TEAM_BLUE,
    POKTOCOL_TEAM_YELLOW,
};

struct poktocol_msg
{
    enum poktocol_commands_types cmd;
    enum poktocol_data_types type;
    union
    {
        uint8_t score;
        enum pokprotocol_team team;
    } data;
};

int poktocol_encode(const struct poktocol_msg *msg, uint8_t *buffer, size_t buffer_size);
int poktocol_decode(const uint8_t *buffer, size_t buffer_size, struct poktocol_msg *msg);

#endif
