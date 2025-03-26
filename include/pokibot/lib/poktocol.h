#ifndef POKTOCOL_H
#define POKTOCOL_H
#include <pokibot/lib/pokutils.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

enum poktocol_commands_types {
    POKTOCOL_CMD_TYPE_WRITE,
    POKTOCOL_CMD_TYPE_REQUEST,
};

enum poktocol_data_types {
    POKTOCOL_DATA_TYPE_UI_SCORE,
    POKTOCOL_DATA_TYPE_UI_TEAM,
    POKTOCOL_DATA_TYPE_UI_MATCH_STARTED,
    POKTOCOL_DATA_TYPE_LEGS_POS,
    POKTOCOL_DATA_TYPE_LEGS_DIR,
    POKTOCOL_DATA_TYPE_LEGS_BREAK,
    POKTOCOL_DATA_TYPE_LEGS_WAYPOINTS,
};

enum pokprotocol_team {
    POKTOCOL_TEAM_BLUE,
    POKTOCOL_TEAM_YELLOW,
};

union poktocol_msg_data
{
    uint8_t score;
    enum pokprotocol_team team;
    pos2_t legs_pos;
    float legs_dir;
    bool legs_break;
    struct {
        pos2_t *wps;
        size_t nb_wps;
    } waypoints;
};

struct poktocol_header
{
    enum poktocol_commands_types cmd;
    enum poktocol_data_types type;
};

struct poktocol_msg
{
    struct poktocol_header header;
    union poktocol_msg_data data;
};

int poktocol_encode(const struct poktocol_msg *msg, uint8_t *buffer, size_t buffer_size);
int poktocol_decode_header(const uint8_t *buffer, size_t buffer_size, struct poktocol_header *msg);
int poktocol_decode_data(const uint8_t *buffer, size_t buffer_size, union poktocol_msg_data *data);

#endif
