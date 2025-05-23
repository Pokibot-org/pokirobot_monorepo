#ifndef POKUICOM_H
#define POKUICOM_H
#include "pokibot/lib/poktocol.h"
#include "stdint.h"

enum pokuicom_match_status {
    POKUICOM_MATCH_STATUS_UNKNOWN,
    POKUICOM_MATCH_STATUS_SETUP,
    POKUICOM_MATCH_STATUS_STARTED,
};

void pokuicom_send_score(uint8_t score);
enum pokuicom_match_status pokuicom_get_match_status(void);
int pokuicom_get_team_color(enum pokprotocol_team *color);
int pokuicom_request(enum poktocol_data_types type);

#endif
