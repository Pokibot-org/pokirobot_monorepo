import paho.mqtt.client as mqtt
import time
import logging
import threading
import queue
from abc import ABC

logger = logging.getLogger("msm")

class MqttSimMessengerNode:
    def __init__(self, parent, id=None, name=None) -> None:
        self.parent = parent
        self.id = id
        self.name = name
        self.status = "unknown"
        self.childs : dict[str, dict[int, MqttSimMessengerNode]] = {}
        if parent:
            parent.add_child(self)

    def __repr__(self) -> str:
        return str(self.childs)

    def add_child(self, node):
        if node.name not in self.childs.keys():
            self.childs[node.name] = {}
        self.childs[node.name][int(node.id)] = node

    def _build_upper_topic(self, topic):
        if self.name:
            return self.name + "/" + str(self.id) + "/" + topic
        if self.id:
            return str(self.id) + "/" + topic
        return topic

    def send(self, topic, payload):
        self.parent.send(self._build_upper_topic(topic), payload)

    def subscribe(self, topic):
        self.parent.subscribe(self._build_upper_topic(topic))

    def process_topic(self, topic, payload):
        pass

    def recv_message(self, topic_list, payload):
        logger.debug(f"<{self.name}|{self.id}>: on_message: {topic_list}")
        if len(topic_list) == 0:
            return
        elif len(topic_list) == 1:
            if topic_list[0] == "status":
                self.status = payload
            self.process_topic(topic_list[0], payload)
            return

        child_name = topic_list[0]
        child_id = int(topic_list[1])
        trimmed_topic = topic_list[2:]

        if child_name not in self.childs.keys():
            logger.error(f"Child {child_name} not found for topic list {topic_list}")
            logger.error(f"childs {self.childs}")
            return
        device_dic = self.childs[child_name]

        if child_id not in device_dic.keys():
            logger.error(f"Child id {child_id} not found for topic list {topic_list}")
            logger.error(f"childs {self.childs}")
            return
        device_dic[child_id].recv_message(trimmed_topic, payload)

def on_message(client, userdata, message):
    userdata.on_message(message)

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code.is_failure:
        logger.info(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
    else:
        userdata.on_connect_clbk()
        # we should always subscribe from on_connect callback to be sure
        # our subscribed is persisted across reconnections.
        client.subscribe("msm/+/+/status")

class MqttSimMessengerServer(MqttSimMessengerNode):
    TX_Q_EVENT_DATA = 0
    TX_Q_EVENT_STOP = 1
    def __init__(self, obj_connect_clbk, obj_diconnect_clbk, on_connect_clbk=lambda:()):
        super().__init__(None, '', None)
        self.obj_connect_clbk = obj_connect_clbk
        self.obj_diconnect_clbk = obj_diconnect_clbk
        self.on_connect_clbk = on_connect_clbk

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.user_data_set(self)
        self.client.will_set("sim/status", "terminated")
        self.tx_queue = queue.SimpleQueue()
        self.tx_task = threading.Thread(target=self._tx_task, daemon=True)

    def start(self):
        logger.info("Start")
        self.tx_task.start()
        self.client.connect("127.0.0.1")
        self.client.loop_start()
        self.client.publish("sim/status", "started").wait_for_publish()

    def stop(self):
        self.tx_queue.put((self.TX_Q_EVENT_STOP, None))
        self.tx_task.join()
        self.client.loop_stop()

    def _tx_task(self):
        while 1:
            ev, data = self.tx_queue.get()
            if ev == self.TX_Q_EVENT_DATA:
                topic, payload = data
                self.client.publish(topic, payload).wait_for_publish()
            elif ev == self.TX_Q_EVENT_STOP:
                break

    def on_message(self, message):
        # logger.debug(f"< {message.topic} {message.payload.decode()}")
        split_topic = message.topic.split("/")
        dev_name = split_topic[1]
        dev_id = split_topic[2]
        trimmed_topic = split_topic[3:]
        decoded_payload = message.payload.decode()

        if len(trimmed_topic) == 1 and trimmed_topic[0] == "status":
            if decoded_payload == "connected":
                logger.info(f"Dev connection {dev_name}")
                self.obj_connect_clbk(dev_name, dev_id)
            if decoded_payload == "disconnected":
                logger.info(f"Dev disconnection {dev_name}")
                self.obj_diconnect_clbk(dev_name, dev_id)
        else:
            self.recv_message(split_topic[1:], decoded_payload)

    def send(self, topic, payload):
        topic = "msm/" + topic
        logger.debug(f"> {topic} {payload}")
        self.tx_queue.put((self.TX_Q_EVENT_DATA, (topic, payload)))

    def subscribe(self, topic):
        topic = "msm/" + topic
        self.client.subscribe(topic)
