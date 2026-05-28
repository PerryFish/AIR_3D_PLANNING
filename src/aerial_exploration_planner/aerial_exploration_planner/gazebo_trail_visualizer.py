#!/usr/bin/env python3
import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class GazeboTrailVisualizer(Node):
    def __init__(self):
        super().__init__("gazebo_trail_visualizer")
        self.declare_parameter("pose_topic", "/state_estimation")
        self.declare_parameter("goal_topic", "/aerial_exploration/goal")
        self.declare_parameter("trail_model_prefix", "uav_trail")
        self.declare_parameter("min_distance", 0.8)
        self.declare_parameter("max_points", 300)
        self.declare_parameter("trail_z_offset", 0.0)
        self.declare_parameter("goal_z_offset", 0.0)
        self.declare_parameter("spawn_every_n_seconds", 0.5)
        self.declare_parameter("waypoint_model_name", "current_exploration_goal")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("enable_waypoint_marker", True)
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.state_client = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.reference_frame = self.get_parameter("reference_frame").value
        self.prefix = self.get_parameter("trail_model_prefix").value
        self.min_distance = float(self.get_parameter("min_distance").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.trail_z_offset = float(self.get_parameter("trail_z_offset").value)
        self.goal_z_offset = float(self.get_parameter("goal_z_offset").value)
        self.last_pose = None
        self.logged_pose = False
        self.logged_goal = False
        self.last_spawn_time = self.get_clock().now()
        self.spawn_count = 0
        self.goal_spawned = False
        self.latest_goal = None
        self.create_subscription(Odometry, self.get_parameter("pose_topic").value, self.odom_cb, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.goal_cb, 10)
        self.goal_timer = self.create_timer(0.2, self.update_goal_marker)
        self.get_logger().info(
            f"Gazebo trail visualizer started; pose_topic={self.get_parameter('pose_topic').value}; "
            f"goal_topic={self.get_parameter('goal_topic').value}; trail_z_offset={self.trail_z_offset:.3f}; "
            f"goal_z_offset={self.goal_z_offset:.3f}"
        )

    def odom_cb(self, msg):
        pose = msg.pose.pose
        if not self.logged_pose:
            self.get_logger().info(
                f"First trail pose z={pose.position.z:.3f}; breadcrumb_z={pose.position.z + self.trail_z_offset:.3f}"
            )
            self.logged_pose = True
        now = self.get_clock().now()
        elapsed = (now - self.last_spawn_time).nanoseconds * 1e-9
        current = (pose.position.x, pose.position.y, pose.position.z)
        if self.spawn_count >= self.max_points:
            return
        if self.last_pose is not None and math.dist(current, self.last_pose) < self.min_distance:
            return
        if elapsed < float(self.get_parameter("spawn_every_n_seconds").value):
            return
        if not self.spawn_client.service_is_ready():
            return
        name = f"{self.prefix}_{self.spawn_count:03d}"
        self.spawn_model(name, self.trail_sdf(name), pose.position.x, pose.position.y, pose.position.z + self.trail_z_offset)
        self.spawn_count += 1
        self.last_pose = current
        self.last_spawn_time = now

    def goal_cb(self, msg):
        self.latest_goal = msg.pose
        self.latest_goal.position.z += self.goal_z_offset
        if not self.logged_goal:
            self.get_logger().info(f"First Gazebo goal marker z={self.latest_goal.position.z:.3f}")
            self.logged_goal = True

    def update_goal_marker(self):
        if not self._as_bool(self.get_parameter("enable_waypoint_marker").value):
            return
        if self.latest_goal is None:
            return
        name = self.get_parameter("waypoint_model_name").value
        if not self.goal_spawned:
            if not self.spawn_client.service_is_ready():
                return
            self.spawn_model(name, self.goal_sdf(name), self.latest_goal.position.x, self.latest_goal.position.y, self.latest_goal.position.z)
            self.goal_spawned = True
            return
        if not self.state_client.service_is_ready():
            return
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.reference_frame = self.reference_frame
        req.state.pose = self.latest_goal
        self.state_client.call_async(req)

    def spawn_model(self, name, xml, x, y, z):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.reference_frame = self.reference_frame
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.w = 1.0
        self.spawn_client.call_async(req)

    def trail_sdf(self, name):
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='trail'>
        <geometry><sphere><radius>0.16</radius></sphere></geometry>
        <material><ambient>0.0 1.0 0.25 1</ambient><diffuse>0.0 1.0 0.25 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

    def goal_sdf(self, name):
        return f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='goal'>
        <geometry><sphere><radius>0.35</radius></sphere></geometry>
        <material><ambient>1.0 0.9 0.0 1</ambient><diffuse>1.0 0.9 0.0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).lower() in ("1", "true", "yes", "on")


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTrailVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
