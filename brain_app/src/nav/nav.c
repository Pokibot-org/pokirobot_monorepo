#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <pokibot/poklegscom.h>
#include <pokibot/lib/pokutils.h>
#include <pokibot/poktypes.h>
#include "nav.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nav);

K_EVENT_DEFINE(nav_events);
K_THREAD_STACK_DEFINE(nav_thread_stack, 2048);
static struct k_thread nav_thread;

pos2_t sensivity = {
    .x = 0.01f,
    .y = 0.01f,
    .a = DEG_TO_RAD(2),
};

pos2_t target_pos = {-10000.0, -10000.0f};

int nav_set_pos(const pos2_t *pos){
    return poklegscom_set_pos(pos);
}

int nav_go_to(const pos2_t *pos, k_timeout_t timeout)
{
    k_event_clear(&nav_events, 0xFFFFFFFF);
    target_pos = *pos;
    poklegscom_set_break(0);
    poklegscom_set_waypoints(pos, 1);
    return 0;
}

void nav_wait_events(uint32_t *events)
{
    k_event_wait(&nav_events, 0xFFFFFFFF, false, K_FOREVER);
}

void nav_thread_task(void *arg0, void *arg1, void *arg2) {
    while (1) {
        pos2_t current_pos;
        poklegscom_get_pos(&current_pos);
        pos2_t diff = pos2_abs(pos2_diff(current_pos, target_pos));
        if (POS2_COMPARE(diff, < , sensivity)) {
            LOG_INF("Target reached");
            k_event_set(&nav_events, NAV_EVENT_DESTINATION_REACHED);
        }
        k_sleep(K_MSEC(1000/25));
    }
}

int nav_init(void) {
    poklegscom_set_break(1);
    k_tid_t thread_id = k_thread_create(&nav_thread, nav_thread_stack, K_THREAD_STACK_SIZEOF(nav_thread_stack),
										nav_thread_task, NULL, NULL, NULL, 10, 0, K_NO_WAIT);
	k_thread_name_set(thread_id, "network");
	return 0;
}

SYS_INIT(nav_init, APPLICATION, 0);
