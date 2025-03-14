#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <pokibot/poklegscom.h>
#include <pokibot/lib/pokutils.h>
#include <pokibot/poktypes.h>
#include <pokibot/drivers/lidar.h>
#include "nav.h"
#include "zephyr/device.h"
#include "zephyr/devicetree.h"
#include <stddef.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nav);

const struct device *lidar_dev = DEVICE_DT_GET(DT_ALIAS(mainlidar));

K_EVENT_DEFINE(nav_events);
K_THREAD_STACK_DEFINE(nav_workq_stack, 2048);
static struct k_work_q nav_workq;

static void check_target_reached_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(check_target_reached_work, check_target_reached_work_handler);

static pos2_t target_pos;

pos2_t sensivity = {
    .x = 0.01f,
    .y = 0.01f,
    .a = DEG_TO_RAD(2),
};

int nav_set_pos(const pos2_t *pos){
    return poklegscom_set_pos(pos);
}

int nav_go_to(const pos2_t *pos, k_timeout_t timeout)
{
    k_event_clear(&nav_events, 0xFFFFFFFF);
    target_pos = *pos;
    poklegscom_set_break(0);
    poklegscom_set_waypoints(pos, 1);
    k_work_schedule(&check_target_reached_work, K_NO_WAIT);
    return 0;
}

void nav_wait_events(uint32_t *events)
{
    k_event_wait(&nav_events, 0xFFFFFFFF, false, K_FOREVER);
}

static void check_target_reached_work_handler(struct k_work *work) {
    pos2_t current_pos;
    poklegscom_get_pos(&current_pos);
    pos2_t diff = pos2_abs(pos2_diff(current_pos, target_pos));
    if (POS2_COMPARE(diff, < , sensivity)) {
        LOG_INF("Target reached");
        k_event_set(&nav_events, NAV_EVENT_DESTINATION_REACHED);
        return;
    }
    k_work_reschedule_for_queue(&nav_workq, k_work_delayable_from_work(work), K_MSEC(1000/25));
}

void lidar_callback(const struct lidar_point *points, size_t nb_points, void* user_data)
{
    LOG_INF("Lidar clbk");
}

int nav_init(void) {
   	static struct k_work_queue_config cfg = {
		.name = "nav_workq",
	};
    poklegscom_set_break(1);
    k_work_queue_start(&nav_workq, nav_workq_stack,
			   K_THREAD_STACK_SIZEOF(nav_workq_stack),
			   8, &cfg);
    static struct lidar_callback lidar_clbk = {
        .clbk = lidar_callback,
    };
    lidar_register_callback(lidar_dev, &lidar_clbk);
    lidar_start(lidar_dev);
	return 0;
}

SYS_INIT(nav_init, APPLICATION, 0);
