#ifndef POKUICOM_H
#define POKUICOM_H
#include <pokibot/lib/poktocol.h>
#include "stdint.h"

void pokuicom_send_score(uint8_t score);
enum pokprotocol_tirette_status pokuicom_get_tirette_status(void);
int pokuicom_get_team_color(enum pokprotocol_team *color);
int pokuicom_request(enum poktocol_data_types type);

#endif
