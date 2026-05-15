#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <pokibot/msm.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include "MQTTClient.h"
#include <cmdline.h>
#include <posix_native_task.h>

LOG_MODULE_REGISTER(msm, CONFIG_MSM_LOG_LEVEL);


#define ADDRESS     "tcp://127.0.0.1:1883"
#define QOS         0
#define TIMEOUT     1000L
#define ROOT_TOPIC "msm/" CONFIG_MSM_DEVICE_NAME "/"

#define MSM_DISPATCH_STACK_SIZE 4096
#define MSM_DISPATCH_PRIO       5
#define MSM_DISPATCH_POLL_MS    5
#define MSM_QUEUE_MAX           32
#define MSM_TOPIC_MAX           128

static char pid_client_id[32];
static char *client_id = NULL;
static char base_topic[64];
static int base_topic_len = 0;

static MQTTClient client;
static volatile MQTTClient_deliveryToken delivered_token;
static sys_slist_t topic_list;

/*
 * Paho-MQTT delivers msg_clbk on its own host pthread. Calling Zephyr APIs
 * from that thread trips native_simulator's nsif_cpu0_irq_raised_from_sw
 * "HW model thread" check. Producer copies the message into a pthread-mutex
 * protected FIFO; a Zephyr thread polls it and dispatches on the SW/CPU side.
 */
struct msm_pending {
    struct msm_pending *next;
    int payload_len;
    char *payload;
    char trimmed_topic[MSM_TOPIC_MAX];
};

static pthread_mutex_t q_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct msm_pending *q_head;
static struct msm_pending *q_tail;
static int q_count;

static K_THREAD_STACK_DEFINE(msm_dispatch_stack, MSM_DISPATCH_STACK_SIZE);
static struct k_thread msm_dispatch_thread;

static void delivered(void *context, MQTTClient_deliveryToken dt)
{
    delivered_token = dt;
}

static int msg_clbk(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    char *trimmed_topic = topicName + base_topic_len;

    struct msm_pending *p = malloc(sizeof(*p));
    if (p == NULL) {
        goto out;
    }
    p->next = NULL;
    p->payload_len = message->payloadlen;
    p->payload = malloc(message->payloadlen > 0 ? message->payloadlen : 1);
    if (p->payload == NULL) {
        free(p);
        goto out;
    }
    if (message->payloadlen > 0) {
        memcpy(p->payload, message->payload, message->payloadlen);
    }
    size_t tl = strlen(trimmed_topic);
    if (tl >= sizeof(p->trimmed_topic)) {
        tl = sizeof(p->trimmed_topic) - 1;
    }
    memcpy(p->trimmed_topic, trimmed_topic, tl);
    p->trimmed_topic[tl] = '\0';

    pthread_mutex_lock(&q_mutex);
    if (q_count >= MSM_QUEUE_MAX) {
        pthread_mutex_unlock(&q_mutex);
        free(p->payload);
        free(p);
        fprintf(stderr, "msm: dispatch queue full, dropping message\n");
        goto out;
    }
    if (q_tail != NULL) {
        q_tail->next = p;
    } else {
        q_head = p;
    }
    q_tail = p;
    q_count++;
    pthread_mutex_unlock(&q_mutex);

out:
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

static void conn_lost(void *context, char *cause)
{
    /* Runs on Paho's host pthread - do not use LOG_*. */
    fprintf(stderr, "msm: connection lost: %s\n", cause ? cause : "(null)");
}

static void msm_dispatch_loop(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (true) {
        struct msm_pending *p;

        pthread_mutex_lock(&q_mutex);
        p = q_head;
        if (p != NULL) {
            q_head = p->next;
            if (q_head == NULL) {
                q_tail = NULL;
            }
            q_count--;
        }
        pthread_mutex_unlock(&q_mutex);

        if (p == NULL) {
            k_msleep(MSM_DISPATCH_POLL_MS);
            continue;
        }

        LOG_DBG("topic: %s", p->trimmed_topic);
        LOG_DBG("message: %.*s", p->payload_len, p->payload);

        sys_snode_t *cur;
        SYS_SLIST_FOR_EACH_NODE(&topic_list, cur) {
            struct msm_topic *topic = CONTAINER_OF(cur, struct msm_topic, _node);
            if (strcmp(p->trimmed_topic, topic->name) == 0) {
                topic->clbk(p->payload, p->payload_len, topic->user_data);
            }
        }
        free(p->payload);
        free(p);
    }
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

    k_thread_create(&msm_dispatch_thread, msm_dispatch_stack,
                    K_THREAD_STACK_SIZEOF(msm_dispatch_stack),
                    msm_dispatch_loop, NULL, NULL, NULL,
                    MSM_DISPATCH_PRIO, 0, K_NO_WAIT);
    k_thread_name_set(&msm_dispatch_thread, "msm_dispatch");

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
    LOG_INF("Connected to MQTT broker at " ADDRESS);

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
