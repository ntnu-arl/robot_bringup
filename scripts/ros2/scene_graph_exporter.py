#!/usr/bin/env python3
"""Export Jazzy Hydra graphs as portable, complete JSON snapshots."""

import json
import uuid
from pathlib import Path

import rclpy
from spark_dsg import _dsg_bindings as spark_dsg
import yaml
import zmq
from hydra_msgs.msg import DsgUpdate
from rclpy.node import Node


class SceneGraphExporter(Node):
    """Keep Hydra's binary format and Python bindings on the Jazzy side."""

    def __init__(self) -> None:
        """Create the graph subscriber and a bounded snapshot publisher."""
        super().__init__("scene_graph_exporter")
        endpoint = self.declare_parameter("endpoint", "tcp://127.0.0.1:8002").value
        topic = self.declare_parameter("input_topic", "/hydra/backend/dsg").value
        period = self.declare_parameter("publish_period", 1.0).value
        label_path = self.declare_parameter("labelspace_path", "").value
        labels = yaml.safe_load(Path(label_path).read_text()) if label_path else {}
        self.labels = {entry["label"]: entry["name"] for entry in labels.get("label_names", [])}
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(endpoint)
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.pending: DsgUpdate | None = None
        self.snapshot: str | None = None
        self.create_subscription(DsgUpdate, topic, self.on_graph, 1)
        # Repeat the latest full snapshot so late or restarted subscribers recover.
        self.create_timer(period, self.publish_snapshot)
        self.get_logger().info(f"Exporting {topic} to {endpoint}")

    def on_graph(self, message: DsgUpdate) -> None:
        """Retain the newest full update without decoding every backend update.

        :param message: Hydra's complete serialized scene graph.
        :return: None.
        """
        if message.full_update:
            self.pending = message
        else:
            self.get_logger().warning("Ignoring incremental graph; exporter requires full snapshots")

    def publish_snapshot(self) -> None:
        """Decode at most one graph per interval and publish a complete snapshot.

        :return: None.
        """
        if self.pending is not None:
            message, self.pending = self.pending, None
            try:
                graph = spark_dsg.DynamicSceneGraph.from_binary(bytes(message.layer_contents))
                nodes = []
                for node in graph.nodes:
                    attrs = node.attributes
                    item = {
                        # Strings preserve uint64 IDs across JSON consumers.
                        "id": str(node.id.value), "symbol": node.id.str(),
                        "layer": node.layer.layer, "partition": node.layer.partition,
                        "position": attrs.position.tolist(),
                        "active": attrs.is_active,
                    }
                    if isinstance(attrs, spark_dsg.SemanticNodeAttributes):
                        item.update(label_id=int(attrs.semantic_label),
                                    label=self.labels.get(attrs.semantic_label, attrs.name))
                        if attrs.bounding_box.is_valid():
                            box = attrs.bounding_box
                            item["bounding_box"] = {
                                "center": box.world_P_center.tolist(),
                                "dimensions": box.dimensions.tolist(),
                                "rotation": box.world_R_center.tolist(),
                            }
                    nodes.append(item)
                self.sequence += 1
                snapshot = {
                    "schema": "agentic_uas.scene_graph", "version": 1,
                    "session": self.session, "sequence": self.sequence,
                    "stamp_ns": message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
                    "frame_id": message.header.frame_id, "full_update": True,
                    "nodes": nodes,
                    "edges": [{"source": str(edge.source), "target": str(edge.target)}
                              for edge in graph.edges],
                }
                self.snapshot = json.dumps(snapshot, allow_nan=False, separators=(",", ":"))
                objects = sum("label_id" in node for node in nodes)
                self.get_logger().info(
                    f"Graph {self.sequence}: {len(nodes)} nodes, {objects} semantic objects, "
                    f"{len(snapshot['edges'])} edges in {snapshot['frame_id']}",
                    throttle_duration_sec=10.0,
                )
            except (RuntimeError, ValueError, TypeError) as error:
                self.get_logger().error(f"Cannot decode graph: {error}")
        if self.snapshot is not None:
            self.socket.send_string(self.snapshot)

    def destroy_node(self) -> None:
        """Close transport resources before destroying the ROS node.

        :return: None.
        """
        self.socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main() -> None:
    """Run the Jazzy exporter.

    :return: None.
    """
    rclpy.init()
    node = SceneGraphExporter()
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
