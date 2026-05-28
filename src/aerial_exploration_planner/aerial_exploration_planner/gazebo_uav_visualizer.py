#!/usr/bin/env python3
from pathlib import Path

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class GazeboUavVisualizer(Node):
    def __init__(self):
        super().__init__("gazebo_uav_visualizer")
        self.declare_parameter("pose_topic", "/state_estimation")
        self.declare_parameter("model_name", "simple_uav")
        self.declare_parameter("model_sdf_path", "")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("visual_z_offset", 0.25)
        self.declare_parameter("spawn_x", -9.0)
        self.declare_parameter("spawn_y", -9.0)
        self.declare_parameter("spawn_z", 1.75)
        self.declare_parameter("update_rate", 15.0)
        self.model_name = self.get_parameter("model_name").value
        self.reference_frame = self.get_parameter("reference_frame").value
        self.visual_z_offset = float(self.get_parameter("visual_z_offset").value)
        self.latest_pose = None
        self.spawned = False
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.create_subscription(Odometry, self.get_parameter("pose_topic").value, self.odom_cb, 10)
        self.spawn_timer = self.create_timer(0.5, self.try_spawn)
        self.update_timer = self.create_timer(1.0 / float(self.get_parameter("update_rate").value), self.update_model)
        self.get_logger().info(f"Gazebo UAV visualizer following {self.get_parameter('pose_topic').value}")

    def odom_cb(self, msg):
        self.latest_pose = msg.pose.pose

    def try_spawn(self):
        if self.spawned or not self.spawn_client.service_is_ready():
            return
        sdf_path = self._model_sdf_path()
        if not sdf_path.exists():
            self.get_logger().error(f"simple_uav model SDF not found: {sdf_path}")
            return
        req = SpawnEntity.Request()
        req.name = self.model_name
        req.xml = sdf_path.read_text()
        req.robot_namespace = ""
        req.reference_frame = self.reference_frame
        req.initial_pose.position.x = float(self.get_parameter("spawn_x").value)
        req.initial_pose.position.y = float(self.get_parameter("spawn_y").value)
        req.initial_pose.position.z = float(self.get_parameter("spawn_z").value)
        req.initial_pose.orientation.w = 1.0
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_done)

    def spawn_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f"spawn simple_uav failed: {exc}")
            return
        if result.success or "already exist" in result.status_message.lower():
            self.spawned = True
            self.spawn_timer.cancel()
            self.get_logger().info(f"Gazebo model ready: {self.model_name}")
        else:
            self.get_logger().warning(f"spawn simple_uav rejected: {result.status_message}")

    def update_model(self):
        if not self.spawned or self.latest_pose is None or not self.state_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.model_name
        req.state.reference_frame = self.reference_frame
        req.state.pose = Pose()
        req.state.pose.position.x = self.latest_pose.position.x
        req.state.pose.position.y = self.latest_pose.position.y
        req.state.pose.position.z = self.latest_pose.position.z + self.visual_z_offset
        req.state.pose.orientation = self.latest_pose.orientation
        self.state_client.call_async(req)

    def _model_sdf_path(self):
        configured = self.get_parameter("model_sdf_path").value
        if configured:
            return Path(configured)
        return Path(__file__).resolve().parents[4] / "share" / "aerial_exploration_planner" / "models" / "simple_uav" / "model.sdf"


def main(args=None):
    rclpy.init(args=args)
    node = GazeboUavVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
