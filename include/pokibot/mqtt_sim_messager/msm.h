#ifndef MQTT_SIM_MESSAGER_H
#define MQTT_SIM_MESSAGER_H
#include <zephyr/sys/slist.h>

struct msm_topic {
	char *name;
	void (*clbk)(char *data, int len, void *user_data);
	void *user_data;

	/** Internally used field for list handling */
	sys_snode_t _node;
};

void msm_topic_register(struct msm_topic *topic);
int msm_send(const char *topic, char *data);

#endif
