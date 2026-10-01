#!/usr/bin/env python3
"""Keep static TF available to the legacy ROS bridge's volatile subscriber."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class StaticTfRepublisher(Node):
    """Republish the complete static transform set for late bridge subscribers."""

    def __init__(self) -> None:
        super().__init__('static_tf_republisher')
        self.transforms: dict[str, TransformStamped] = {}
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(TFMessage, '/tf_static', qos)
        self.subscription = self.create_subscription(
            TFMessage, '/tf_static', self.receive, qos
        )
        self.timer = self.create_timer(1.0, self.publish)

    def receive(self, message: TFMessage) -> None:
        """Cache transforms by child frame.

        :param message: Static transforms received from simulation publishers.
        :return: None.
        """
        for transform in message.transforms:
            self.transforms[transform.child_frame_id] = transform

    def publish(self) -> None:
        """Repeat all static transforms together, including the camera extrinsics."""
        if self.transforms:
            self.publisher.publish(TFMessage(transforms=list(self.transforms.values())))


def main() -> None:
    """Run the static TF compatibility republisher."""
    rclpy.init()
    node = StaticTfRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
