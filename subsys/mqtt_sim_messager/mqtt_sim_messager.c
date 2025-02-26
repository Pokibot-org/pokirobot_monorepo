#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/mqtt_sim_messager/msm.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "MQTTClient.h"
#include <cmdline.h>
#include <posix_native_task.h>

LOG_MODULE_REGISTER(msm, CONFIG_MQTT_SIM_MESSAGER_LOG_LEVEL);


#define ADDRESS     "tcp://127.0.0.1:1883" 
#define QOS         0
#define TIMEOUT     1000L
#define ROOT_TOPIC "msm/"

static char pid_client_id[32];
static char *client_id = NULL;
static char base_topic[64];
static int base_topic_len = 0;

static MQTTClient client;
static volatile MQTTClient_deliveryToken delivered_token;
static sys_slist_t topic_list;


static void delivered(void *context, MQTTClient_deliveryToken dt)
{
    LOG_DBG("Message with token value %d delivery confirmed", dt);
    delivered_token = dt;
}

static int msg_clbk(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    LOG_DBG("topic: %s", topicName);
    LOG_DBG("message: %.*s", message->payloadlen, (char*)message->payload);

    char *trimmed_topic = topicName + base_topic_len;

	sys_snode_t *cur;
	SYS_SLIST_FOR_EACH_NODE(&topic_list, cur) {
		struct msm_topic *topic = CONTAINER_OF(cur, struct msm_topic, _node);

		if (strcmp(trimmed_topic, topic->name) == 0) {
			topic->clbk((char*)message->payload,  message->payloadlen, topic->user_data);
		}
	}

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

static void conn_lost(void *context, char *cause)
{
    LOG_ERR("Connection lost");
    LOG_ERR(" cause: %s", cause);
}


void msm_topic_register(struct msm_topic *topic)
{
    char full_topic[128];
    snprintf(full_topic, sizeof(full_topic), "%s%s", base_topic, topic->name);
    
    LOG_DBG("REG %s", full_topic);
    MQTTClient_subscribe(client, full_topic, QOS);
	sys_slist_append(&topic_list, &topic->_node);
}

int msm_send(const char *topic, char *data)
{
    int rc;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = data;
    pubmsg.payloadlen = (int)strlen(data);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;
    MQTTClient_deliveryToken token;

    char full_topic[128];
    snprintf(full_topic, sizeof(full_topic), "%s%s", base_topic, topic);

    MQTTClient_publishMessage(client, full_topic, &pubmsg, &token);
    rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
	return rc;
}

int msm_init(void)
{
    int rc;

    if (client_id == NULL) {
        int pid = getpid();
        snprintf(pid_client_id, sizeof(pid_client_id), "%d", pid);
        client_id = pid_client_id;
    }

    snprintf(base_topic, sizeof(base_topic), ROOT_TOPIC"%s/", client_id);
    base_topic_len = strlen(base_topic);

    MQTTClient_create(&client, ADDRESS, client_id, MQTTCLIENT_PERSISTENCE_NONE, NULL);

    MQTTClient_setCallbacks(client, NULL, conn_lost, msg_clbk, delivered);

    static MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.cleansession = 1;
    conn_opts.keepAliveInterval = 20;
    
    // Define the last will message and topic
    static char lw_topic[128];
    snprintf(lw_topic, sizeof(lw_topic), "%s%s", base_topic, "status");
    static MQTTClient_willOptions wopts =  MQTTClient_willOptions_initializer;
    conn_opts.will = &wopts;
    conn_opts.will->message = "disconnected";
    conn_opts.will->qos = 1;
    conn_opts.will->retained = 0;
    conn_opts.will->topicName = lw_topic;
 
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        LOG_INF("Failed to connect, return code %d", rc);
        exit(EXIT_FAILURE);
    }
    LOG_INF("Connected to MQTT broker at %s", ADDRESS);

    // MQTTClient_disconnect(client, 10000);
    // MQTTClient_destroy(&client);
    msm_send("status", "connected");
	return 0;
}

SYS_INIT(msm_init, POST_KERNEL, 5);

static void msm_cleanup(void)
{
    msm_send("status", "disconnected");
}

static void msm_options(void)
{
	static struct args_struct_t msm_options[] = {
		{ .option = "msmcid",
		  .name = "msm_client_id",
		  .type = 's',
		  .dest = (void *)&client_id,
		  .descript = "client id"
		},
		ARG_TABLE_ENDMARKER
	};

	native_add_command_line_opts(msm_options);
}


NATIVE_TASK(msm_options, PRE_BOOT_1, 1);
NATIVE_TASK(msm_cleanup, ON_EXIT, 1);