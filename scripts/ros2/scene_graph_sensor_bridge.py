#!/usr/bin/env python3
"""Relay standard ROS CDR payloads between isolated Humble and Jazzy domains."""

import functools

import rclpy
import zmq
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage


class SceneGraphSensorBridge(Node):
    """Relay an explicit topic allowlist and retain configured late-join data."""

    def __init__(self) -> None:
        """Configure one relay endpoint for the distribution of its container."""
        super().__init__("scene_graph_sensor_bridge")
        role = self.declare_parameter("role", "receive").value
        endpoint = self.declare_parameter("endpoint", "tcp://127.0.0.1:8003").value
        topics = self.declare_parameter("topics", ["/tf", "/tf_static", "/clock"]).value
        types = self.declare_parameter("types", ["tf2_msgs/msg/TFMessage",
                                                "tf2_msgs/msg/TFMessage",
                                                "rosgraph_msgs/msg/Clock"]).value
        latched = self.declare_parameter("latched_topics", ["/tf_static"]).value
        transient = self.declare_parameter("transient_local_topics", ["/tf_static"]).value
        if role not in ("send", "receive") or len(topics) != len(types):
            raise ValueError("role must be send/receive, with a message type for every topic")
        self.message_types = {topic: get_message(kind) for topic, kind in zip(topics, types)}
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB if role == "send" else zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.retained: dict[str, bytes] = {}
        self.static_transforms = {}
        self.received: set[str] = set()
        if role == "send":
            self.socket.setsockopt(zmq.SNDHWM, 30)
            self.socket.bind(endpoint)
            for topic, kind in self.message_types.items():
                qos = (QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                       if topic in transient else qos_profile_sensor_data)
                self.create_subscription(
                    kind, topic, functools.partial(self.send, topic, topic in latched),
                    qos, raw=True)
            self.create_timer(1.0, self.repeat_retained)
        else:
            self.socket.setsockopt(zmq.RCVHWM, 30)
            self.socket.setsockopt(zmq.SUBSCRIBE, b"")
            self.socket.connect(endpoint)
            self.topic_publishers = {
                topic: self.create_publisher(
                    kind, topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    if topic in latched else QoSProfile(depth=5))
                for topic, kind in self.message_types.items()
            }
            self.create_timer(0.005, self.receive)
        self.get_logger().info(f"Topic relay {role}: {endpoint}; topics: {topics}")

    def send(self, topic: str, latched: bool, data: bytes) -> None:
        """Forward raw CDR and combine static transforms from independent publishers.

        :param topic: Allowed source topic.
        :param latched: Whether this topic must survive late receiver starts.
        :param data: Serialized ROS message, without image encoding conversion.
        :return: None.
        """
        if topic == "/tf_static":
            message = deserialize_message(data, TFMessage)
            for transform in message.transforms:
                self.static_transforms[transform.child_frame_id] = transform
            data = serialize_message(TFMessage(transforms=list(self.static_transforms.values())))
        if latched:
            self.retained[topic] = data
        self.socket.send_multipart([topic.encode(), data])

    def repeat_retained(self) -> None:
        """Replay retained messages for reconnecting subscribers.

        :return: None.
        """
        for topic, data in self.retained.items():
            self.socket.send_multipart([topic.encode(), data])

    def receive(self) -> None:
        """Drain a bounded batch without blocking the ROS executor.

        :return: None.
        """
        for _ in range(30):
            try:
                topic_bytes, data = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            topic = topic_bytes.decode()
            if topic not in self.message_types:
                continue
            try:
                # Standard message definitions match: publish CDR directly rather
                # than copying large image arrays through Python message objects.
                self.topic_publishers[topic].publish(data)
            except (ValueError, RuntimeError) as error:
                self.get_logger().error(f"Invalid topic payload on {topic}: {error}")
                continue
            if topic not in self.received:
                self.received.add(topic)
                self.get_logger().info(f"Receiving {topic}")

    def destroy_node(self) -> None:
        """Close the relay socket and ROS resources.

        :return: None.
        """
        self.socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main() -> None:
    """Run one half of the topic relay.

    :return: None.
    """
    rclpy.init()
    node = SceneGraphSensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
