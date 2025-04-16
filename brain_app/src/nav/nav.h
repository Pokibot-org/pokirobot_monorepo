#ifndef NAV_H
#define NAV_H

#include <zephyr/kernel.h>
#include "pokibot/poktypes.h"
#include <stdint.h>

#define NAV_EVENT_DESTINATION_REACHED BIT(0)
#define NAV_EVENT_TIMEOUT             BIT(1)
#define NAV_EVENT_CANCELED            BIT(2)
#define ALL_NAV_EVENTS                (NAV_EVENT_DESTINATION_REACHED | NAV_EVENT_TIMEOUT | NAV_EVENT_CANCELED)

int nav_set_pos(const pos2_t *pos);
int nav_set_break(bool status);
int nav_go_to(const pos2_t *pos, k_timeout_t timeout);
int nav_go_to_direct(const pos2_t *pos, k_timeout_t timeout);
void nav_cancel(void);
void nav_wait_events(uint32_t *events);

#endif
