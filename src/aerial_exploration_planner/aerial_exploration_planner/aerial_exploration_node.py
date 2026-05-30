import json
import math
from collections import deque
from heapq import heappop, heappush

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .grid_model import GridSpec, grid_to_world, ground_to_occupied_voxels, make_ground_footprint, world_to_grid


class AerialExplorationNode(Node):
    def __init__(self):
        super().__init__("aerial_exploration_node")
        self.declare_parameter("exploration.plan_period_sec", 0.5)
        self.declare_parameter("exploration.done_threshold", 0.93)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid.x_cells", 20)
        self.declare_parameter("grid.y_cells", 20)
        self.declare_parameter("grid.z_cells", 6)
        self.declare_parameter("grid.resolution", 1.0)
        self.declare_parameter("grid.origin_x", 0.0)
        self.declare_parameter("grid.origin_y", 0.0)
        self.declare_parameter("dense50.ground_footprint_occupancy_ratio", 0.50)
        self.declare_parameter("garage_v1.start_pose.x", -9.0)
        self.declare_parameter("garage_v1.start_pose.y", -9.0)
        self.declare_parameter("garage_v1.start_pose.z", 1.4)
        self.declare_parameter("garage_v1.start_pose.yaw", 0.0)
        self.declare_parameter("sensor_mapping.environment_model", "dense50")
        self.declare_parameter("aerial_corridor_mode", True)
        self.declare_parameter("aerial_corridor_min_z", 0.8)
        self.declare_parameter("aerial_corridor_max_z", 2.2)
        self.declare_parameter("aerial_corridor_default_z", 1.4)
        self.declare_parameter("aerial_corridor_clearance", 0.25)
        self.declare_parameter("avoid_flying_above_obstacle_top", True)
        self.declare_parameter("max_above_obstacle_margin", 0.2)
        self.declare_parameter("exploration.goal_reached_radius", 0.8)
        self.declare_parameter("exploration.sensor_mapping_patrol_every_n_goals", 3)
        self.declare_parameter("exploration.min_coverage_improvement_rate", 0.0)
        self.declare_parameter("exploration.stuck_window_sec", 8.0)
        self.declare_parameter("exploration.stuck_distance_threshold", 0.25)
        self.declare_parameter("exploration.failed_goal_blacklist_radius", 0.8)
        self.declare_parameter("exploration.frontier_revisit_cooldown_sec", 20.0)
        self.declare_parameter("exploration.backtrack_enabled", False)
        self.declare_parameter("exploration.backtrack_after_stuck_sec", 8.0)
        self.declare_parameter("exploration.backtrack_distance", 1.5)
        self.declare_parameter("exploration.information_gain_weight", 1.0)
        self.declare_parameter("exploration.distance_cost_weight", 0.5)
        self.declare_parameter("exploration.turning_cost_weight", 0.0)
        self.declare_parameter("exploration.narrow_passage_bonus", 0.0)
        self.declare_parameter("exploration.unexplored_area_bonus", 1.0)
        self.declare_parameter("exploration.loop_closure_bonus", 0.0)
        self.declare_parameter("planner.max_goal_distance", 0.0)
        self.declare_parameter("planner.min_goal_distance", 0.0)
        self.declare_parameter("altitude_planning.adaptive_z_enabled", False)
        self.declare_parameter("altitude_planning.z_levels", [1.4])
        self.declare_parameter("altitude_planning.vertical_step_cost", 0.4)
        self.declare_parameter("altitude_planning.prefer_current_altitude_weight", 0.5)
        self.declare_parameter("altitude_planning.information_gain_z_weight", 1.0)
        self.done_threshold = float(self.get_parameter("exploration.done_threshold").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.environment_model = str(self.get_parameter("sensor_mapping.environment_model").value)
        self.spec = GridSpec(
            int(self.get_parameter("grid.x_cells").value),
            int(self.get_parameter("grid.y_cells").value),
            int(self.get_parameter("grid.z_cells").value),
            float(self.get_parameter("grid.resolution").value),
            float(self.get_parameter("dense50.ground_footprint_occupancy_ratio").value),
            float(self.get_parameter("grid.origin_x").value),
            float(self.get_parameter("grid.origin_y").value),
        )
        self.corridor_min_z = float(self.get_parameter("aerial_corridor_min_z").value)
        self.corridor_max_z = float(self.get_parameter("aerial_corridor_max_z").value)
        self.corridor_default_z = self._clamp_z(float(self.get_parameter("aerial_corridor_default_z").value))
        self.start_pose = (
            float(self.get_parameter("garage_v1.start_pose.x").value),
            float(self.get_parameter("garage_v1.start_pose.y").value),
            self._clamp_z(float(self.get_parameter("garage_v1.start_pose.z").value)),
            float(self.get_parameter("garage_v1.start_pose.yaw").value),
        )
        self.corridor_clearance = float(self.get_parameter("aerial_corridor_clearance").value)
        self.goal_reached_radius = float(self.get_parameter("exploration.goal_reached_radius").value)
        self.sensor_patrol_every = int(self.get_parameter("exploration.sensor_mapping_patrol_every_n_goals").value)
        self.min_coverage_rate = float(self.get_parameter("exploration.min_coverage_improvement_rate").value)
        self.stuck_window_sec = float(self.get_parameter("exploration.stuck_window_sec").value)
        self.stuck_distance_threshold = float(self.get_parameter("exploration.stuck_distance_threshold").value)
        self.revisit_cooldown_sec = float(self.get_parameter("exploration.frontier_revisit_cooldown_sec").value)
        self.backtrack_enabled = self._as_bool(self.get_parameter("exploration.backtrack_enabled").value)
        self.backtrack_after_stuck_sec = float(self.get_parameter("exploration.backtrack_after_stuck_sec").value)
        self.backtrack_distance = float(self.get_parameter("exploration.backtrack_distance").value)
        self.info_gain_weight = float(self.get_parameter("exploration.information_gain_weight").value)
        self.distance_cost_weight = float(self.get_parameter("exploration.distance_cost_weight").value)
        self.turning_cost_weight = float(self.get_parameter("exploration.turning_cost_weight").value)
        self.narrow_passage_bonus = float(self.get_parameter("exploration.narrow_passage_bonus").value)
        self.unexplored_area_bonus = float(self.get_parameter("exploration.unexplored_area_bonus").value)
        self.loop_closure_bonus = float(self.get_parameter("exploration.loop_closure_bonus").value)
        self.max_goal_distance = float(self.get_parameter("planner.max_goal_distance").value)
        self.min_goal_distance = float(self.get_parameter("planner.min_goal_distance").value)
        self.adaptive_z_enabled = self._as_bool(self.get_parameter("altitude_planning.adaptive_z_enabled").value)
        self.z_levels = [self._clamp_z(float(z)) for z in self.get_parameter("altitude_planning.z_levels").value]
        if not self.z_levels:
            self.z_levels = [self.corridor_default_z]
        self.vertical_step_cost = float(self.get_parameter("altitude_planning.vertical_step_cost").value)
        self.prefer_current_altitude_weight = float(self.get_parameter("altitude_planning.prefer_current_altitude_weight").value)
        self.information_gain_z_weight = float(self.get_parameter("altitude_planning.information_gain_z_weight").value)
        self.ground_occupied = make_ground_footprint(self.spec, self.environment_model)
        self.occupied_voxels = ground_to_occupied_voxels(self.spec, self.ground_occupied)
        self.pose = None
        self.map_state = None
        self.current_goal = None
        self.goal_index = 0
        self.pose_history = deque(maxlen=600)
        self.coverage_history = deque(maxlen=200)
        self.visited_goal_cells = {}
        self.frontier_blacklist = {}
        self.branch_points = deque(maxlen=80)
        self.stuck_events = 0
        self.backtrack_events = 0
        self.dead_end_count = 0
        self.returned_to_branch_count = 0
        self.failed_goals = 0
        self.frontier_goals = 0
        self.goal_source = "startup"
        self.active_goal_source = "startup"
        self.frontier_cluster_id = -1
        self.adaptive_z_goal_count = 0
        self.z_values_seen = []
        self.last_selected_cell = None
        self.coverage_pub = self.create_publisher(Float32, "/aerial_exploration/coverage", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/aerial_exploration/goal", 10)
        self.planner_state_pub = self.create_publisher(String, "/aerial_exploration/planner_state", 10)
        self.path_pub = self.create_publisher(Path, "/aerial_exploration/path", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, "/aerial_exploration/frontiers", 10)
        self.viewpoint_pub = self.create_publisher(MarkerArray, "/aerial_exploration/viewpoints", 10)
        self.selected_goal_pub = self.create_publisher(Marker, "/aerial_exploration/selected_goal_marker", 10)
        self.start_marker_pub = self.create_publisher(Marker, "/exploration/start_pose_marker", 10)
        self.coverage_marker_pub = self.create_publisher(Marker, "/aerial_exploration/coverage_marker", 10)
        self.ground_marker_pub = self.create_publisher(Marker, "/aerial_exploration/ground_footprint_marker", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Odometry, "/state_estimation", self.odom_cb, 10)
        self.create_subscription(str_msg(), "/aerial_exploration/map_state", self.map_cb, 10)
        self.timer = self.create_timer(float(self.get_parameter("exploration.plan_period_sec").value), self.tick)
        self.get_logger().info(
            "Pose/map-driven aerial exploration node started "
            f"with corridor z=[{self.corridor_min_z:.2f}, {self.corridor_max_z:.2f}], "
            f"default_z={self.corridor_default_z:.2f}"
        )

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)
        now = self._now_sec()
        self.pose_history.append((now, self.pose))

    def map_cb(self, msg):
        self.map_state = json.loads(msg.data)

    def tick(self):
        if not self.map_state:
            return
        coverage = float(self.map_state.get("observed_coverage", self.map_state.get("coverage", 0.0)))
        synthetic_coverage = float(self.map_state.get("synthetic_coverage", self.map_state.get("coverage", coverage)))
        now = self._now_sec()
        self.coverage_history.append((now, coverage))
        coverage_msg = Float32()
        coverage_msg.data = coverage
        self.coverage_pub.publish(coverage_msg)
        self.coverage_marker_pub.publish(
            self._text_marker("coverage", 9001, f"observed_coverage={coverage:.3f} synthetic={synthetic_coverage:.3f}", (0.0, -11.0, 3.0))
        )
        gr = float(self.map_state.get("ground_footprint_occupancy_ratio", 0.0))
        env = self.map_state.get("environment_model", self.environment_model)
        self.ground_marker_pub.publish(self._text_marker("ground_footprint", 9002, f"{env} fallback footprint={gr:.3f}", (0.0, -11.0, 2.5)))
        if self.pose is None or coverage >= self.done_threshold:
            self.start_marker_pub.publish(self._start_pose_marker())
            self._publish_planner_state(coverage)
            return
        self.z_values_seen.append(self.pose[2])
        if self._is_stuck(now, coverage):
            self._handle_stuck(now)
        if self.current_goal is None or math.dist(self.pose, self.current_goal) <= self.goal_reached_radius:
            if self.current_goal is not None and self.active_goal_source == "backtrack_to_branch":
                self.returned_to_branch_count += 1
            if self.current_goal is not None and self.last_selected_cell is not None:
                self.visited_goal_cells[self.last_selected_cell] = now
            goal = self._select_goal()
            if goal is None:
                self._publish_planner_state(coverage)
                return
            self.current_goal = goal
            self.goal_index += 1
            self.goal_pub.publish(self._goal_pose(goal))
        path = self._path_to_goal(self.current_goal)
        self.path_pub.publish(path)
        self.frontier_pub.publish(self._frontier_markers())
        self.viewpoint_pub.publish(self._viewpoint_markers())
        self.selected_goal_pub.publish(self._goal_marker(self.current_goal))
        self.start_marker_pub.publish(self._start_pose_marker())
        self._update_branch_points()
        self._publish_planner_state(coverage)

    def _select_goal(self):
        frontiers = self.map_state.get("frontier_cells", [])
        if "simulated" in str(self.map_state.get("mapping_source", "")) and self.sensor_patrol_every > 0:
            if self.goal_index % self.sensor_patrol_every == 0:
                return self._nearest_free_lawnmower_goal()
        if not frontiers:
            return self._lawnmower_goal()
        candidates = []
        for cell in frontiers:
            cell = tuple(cell)
            if self._is_blacklisted(cell):
                continue
            p = self._corridor_point_from_cell(cell)
            if self.is_point_in_aerial_free_space(*p):
                dist = math.dist(p, self.pose)
                if self.min_goal_distance > 0.0 and dist < self.min_goal_distance:
                    continue
                if self.max_goal_distance > 0.0 and dist > self.max_goal_distance and not self._coverage_stagnant():
                    continue
                candidates.append((cell, p))
        if not candidates:
            self.dead_end_count += 1
            self.goal_source = "nearest_free_lawnmower_no_frontier_candidate"
            self.frontier_cluster_id = -1
            return self._nearest_free_lawnmower_goal()
        cluster_candidates = self._rank_frontier_clusters(candidates)
        candidates = cluster_candidates if cluster_candidates else sorted(candidates, key=lambda item: self._frontier_score(item[0], item[1]), reverse=True)
        for cell, candidate in candidates:
            if self.is_segment_collision_free_3d(self.pose, candidate):
                self.last_selected_cell = cell
                self.frontier_goals += 1
                self.goal_source = "frontier_cluster"
                self.active_goal_source = self.goal_source
                return candidate
        self.failed_goals += 1
        cell, candidate = candidates[0]
        self._blacklist_cell(cell)
        self.last_selected_cell = cell
        self.goal_source = "frontier_fallback_after_collision"
        self.active_goal_source = self.goal_source
        return candidate

    def _rank_frontier_clusters(self, candidates):
        clusters = {}
        for cell, point in candidates:
            angle = math.atan2(point[1] - self.pose[1], point[0] - self.pose[0]) if self.pose else 0.0
            cluster_id = int(((angle + math.pi) / (math.pi / 4.0))) % 8
            clusters.setdefault(cluster_id, []).append((cell, point))
        ranked = []
        for cluster_id, items in clusters.items():
            best_cell, best_point = max(items, key=lambda item: self._frontier_score(item[0], item[1]))
            info_gain = sum(self._nearby_unknown_score(cell, radius=1) for cell, _ in items)
            distance = math.dist(best_point, self.pose) if self.pose else 0.0
            start_distance = math.dist(best_point, self.start_pose[:3])
            revisit = sum(1 for cell, _ in items if self._recently_visited(cell))
            score = info_gain + 0.20 * start_distance - 0.35 * distance - 2.0 * revisit + min(len(items), 20) * 0.1
            ranked.append((score, cluster_id, best_cell, best_point))
        ranked.sort(reverse=True)
        if ranked:
            self.frontier_cluster_id = ranked[0][1]
        return [(cell, point) for _, _, cell, point in ranked]

    def _frontier_score(self, cell, point):
        distance = max(0.001, math.dist(point, self.pose))
        heading_change = self._heading_change(point)
        info_gain = self._nearby_unknown_score(cell, radius=2)
        narrow_score = self._narrow_passage_score(cell)
        revisit_penalty = 1.0 if self._recently_visited(cell) else 0.0
        far_bonus = 1.0 if self._coverage_stagnant() else 0.0
        loop_bonus = self._loop_closure_score(point)
        return (
            self.info_gain_weight * info_gain
            + self.unexplored_area_bonus * info_gain
            + self.narrow_passage_bonus * narrow_score
            + self.loop_closure_bonus * loop_bonus
            + far_bonus * min(distance, 6.0)
            - self.distance_cost_weight * distance
            - self.turning_cost_weight * heading_change
            - 4.0 * revisit_penalty
        )

    def _nearby_unknown_score(self, cell, radius):
        frontiers = {tuple(c) for c in self.map_state.get("frontier_cells", [])}
        score = 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if (cell[0] + dx, cell[1] + dy, cell[2]) in frontiers:
                    score += 1
        return float(score)

    def _narrow_passage_score(self, cell):
        free_neighbors = 0
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            if self._is_free_xy((cell[0] + dx, cell[1] + dy)):
                free_neighbors += 1
        return 1.0 if free_neighbors <= 2 else 0.0

    def _loop_closure_score(self, point):
        if not self.pose_history:
            return 0.0
        historical = [p for _, p in self.pose_history]
        return 1.0 if any(1.0 < math.dist(point, p) < 2.5 for p in historical[:-20]) else 0.0

    def _heading_change(self, point):
        if len(self.pose_history) < 2:
            return 0.0
        a = self.pose_history[-2][1]
        b = self.pose_history[-1][1]
        current = math.atan2(b[1] - a[1], b[0] - a[0])
        desired = math.atan2(point[1] - self.pose[1], point[0] - self.pose[0])
        return abs(math.atan2(math.sin(desired - current), math.cos(desired - current)))

    def _coverage_stagnant(self):
        if self.min_coverage_rate <= 0.0 or len(self.coverage_history) < 2:
            return False
        latest_t, latest_cov = self.coverage_history[-1]
        for old_t, old_cov in self.coverage_history:
            if latest_t - old_t >= self.stuck_window_sec:
                return (latest_cov - old_cov) / max(0.001, latest_t - old_t) < self.min_coverage_rate
        return False

    def _is_stuck(self, now, coverage):
        if not self.current_goal or len(self.pose_history) < 2:
            return False
        old = None
        for t, p in self.pose_history:
            if now - t >= self.stuck_window_sec:
                old = p
        if old is None:
            return False
        return math.dist(old, self.pose) < self.stuck_distance_threshold and self._coverage_stagnant()

    def _handle_stuck(self, now):
        self.stuck_events += 1
        self.failed_goals += 1
        if self.last_selected_cell is not None:
            self._blacklist_cell(self.last_selected_cell)
        if self.backtrack_enabled:
            target = self._backtrack_target()
            if target is not None:
                self.current_goal = target
                self.backtrack_events += 1
                self.dead_end_count += 1
                self.goal_source = "backtrack_to_branch"
                self.active_goal_source = self.goal_source
                self.frontier_cluster_id = -1
                self.goal_pub.publish(self._goal_pose(target))

    def _backtrack_target(self):
        if self.branch_points:
            return self.branch_points[-1]
        if not self.pose_history:
            return None
        for _, p in reversed(self.pose_history):
            if math.dist(p, self.pose) >= self.backtrack_distance and self.is_point_in_aerial_free_space(*p):
                return self._corridor_point(p)
        return None

    def _update_branch_points(self):
        if not self.pose or not self.map_state:
            return
        directions = set()
        for cell in self.map_state.get("frontier_cells", [])[:160]:
            p = self._corridor_point_from_cell(tuple(cell))
            directions.add(int(((math.atan2(p[1] - self.pose[1], p[0] - self.pose[0]) + math.pi) / (math.pi / 4.0))) % 8)
        if len(directions) >= 2 or len(self.map_state.get("frontier_cells", [])) >= 3:
            if not self.branch_points or math.dist(self.branch_points[-1], self.pose) > 1.0:
                self.branch_points.append(self._corridor_point(self.pose))

    def _blacklist_cell(self, cell):
        self.frontier_blacklist[(cell[0], cell[1])] = self._now_sec() + self.revisit_cooldown_sec

    def _is_blacklisted(self, cell):
        key = (cell[0], cell[1])
        until = self.frontier_blacklist.get(key)
        if until is None:
            return False
        if until <= self._now_sec():
            self.frontier_blacklist.pop(key, None)
            return False
        return True

    def _recently_visited(self, cell):
        key = (cell[0], cell[1], cell[2])
        seen = self.visited_goal_cells.get(key)
        return seen is not None and self._now_sec() - seen < self.revisit_cooldown_sec

    def _publish_planner_state(self, coverage):
        msg = String()
        msg.data = json.dumps(
            {
                "environment_model": self.environment_model,
                "observed_coverage": coverage,
                "stuck_events": self.stuck_events,
                "backtrack_events": self.backtrack_events,
                "failed_goals": self.failed_goals,
                "frontier_goals": self.frontier_goals,
                "goal_source": self.goal_source,
                "frontier_cluster_id": self.frontier_cluster_id,
                "branch_point_count": len(self.branch_points),
                "dead_end_count": self.dead_end_count,
                "backtrack_count": self.backtrack_events,
                "returned_to_branch_count": self.returned_to_branch_count,
                "current_region": self._current_region(),
                "start_pose_x": self.start_pose[0],
                "start_pose_y": self.start_pose[1],
                "start_pose_z": self.start_pose[2],
                "start_pose_yaw": self.start_pose[3],
                "distance_from_start": math.dist(self.pose, self.start_pose[:3]) if self.pose else 0.0,
                "adaptive_z_enabled": self.adaptive_z_enabled,
                "adaptive_z_goal_count": self.adaptive_z_goal_count,
                "z_min": min(self.z_values_seen) if self.z_values_seen else self.corridor_default_z,
                "z_max": max(self.z_values_seen) if self.z_values_seen else self.corridor_default_z,
                "blacklisted_frontiers": len(self.frontier_blacklist),
                "branch_points": len(self.branch_points),
            },
            sort_keys=True,
        )
        self.planner_state_pub.publish(msg)

    def _current_region(self):
        if self.pose is None:
            return "unknown"
        d = math.dist(self.pose, self.start_pose[:3])
        if d < 3.0:
            return "entry_start_region"
        if self.branch_points and math.dist(self.pose, self.branch_points[-1]) < 2.0:
            return "branch_region"
        if self.goal_source == "backtrack_to_branch":
            return "backtracking"
        return "garage_corridor"

    def _lawnmower_goal(self):
        cols = [self.spec.origin_x + dx for dx in (-8.0, -4.0, 0.0, 4.0, 8.0)]
        y = self.spec.origin_y + (8.0 if self.goal_index % 2 == 0 else -8.0)
        x = cols[self.goal_index % len(cols)]
        return (x, y, self._best_z_for_xy(x, y))

    def _nearest_free_lawnmower_goal(self):
        base = self._lawnmower_goal()
        if self.is_point_in_aerial_free_space(*base):
            return base
        bx, by, _ = world_to_grid(self.spec, base)
        options = []
        for radius in range(1, max(self.spec.x_cells, self.spec.y_cells)):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    ix, iy = bx + dx, by + dy
                    if 0 <= ix < self.spec.x_cells and 0 <= iy < self.spec.y_cells:
                        p = self._corridor_point_from_cell((ix, iy, 0))
                        if self.is_point_in_aerial_free_space(*p):
                            options.append(p)
            if options:
                options.sort(key=lambda p: math.dist(p, base))
                return options[0]
        return base

    def _path_to_goal(self, goal):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        start = self._corridor_point(self.pose or (-9.0, -9.0, self.corridor_default_z))
        goal = self._corridor_point(goal)
        points = self._plan_corridor_path(start, goal)
        if len(points) < 2:
            points = [start, goal]
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _plan_corridor_path(self, start, goal):
        if self.is_segment_collision_free_3d(start, goal):
            return self._sample_segment(start, goal)
        start_idx = self._nearest_free_xy(world_to_grid(self.spec, start)[:2])
        goal_idx = self._nearest_free_xy(world_to_grid(self.spec, goal)[:2])
        if start_idx is None or goal_idx is None:
            return self._sample_segment(start, goal)
        cells = self._astar_xy(start_idx, goal_idx)
        if not cells:
            return self._sample_segment(start, goal)
        waypoints = [(grid_to_world(self.spec, (ix, iy, 0))[0], grid_to_world(self.spec, (ix, iy, 0))[1], self.corridor_default_z) for ix, iy in cells]
        waypoints[0] = start
        waypoints[-1] = goal
        return self._densify_path(waypoints)

    def _astar_xy(self, start, goal):
        frontier = []
        heappush(frontier, (0.0, start))
        came_from = {start: None}
        cost_so_far = {start: 0.0}
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
        while frontier:
            _, current = heappop(frontier)
            if current == goal:
                break
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._is_free_xy(nxt):
                    continue
                new_cost = cost_so_far[current] + 1.0
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                    heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return []
        cells = []
        current = goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        return cells

    def _nearest_free_xy(self, idx):
        if self._is_free_xy(idx):
            return idx
        for radius in range(1, max(self.spec.x_cells, self.spec.y_cells)):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (idx[0] + dx, idx[1] + dy)
                    if self._is_free_xy(candidate):
                        return candidate
        return None

    def _is_free_xy(self, idx):
        if not (0 <= idx[0] < self.spec.x_cells and 0 <= idx[1] < self.spec.y_cells):
            return False
        point = grid_to_world(self.spec, (idx[0], idx[1], 0))
        return self.is_point_in_aerial_free_space(point[0], point[1], self.corridor_default_z)

    def _sample_segment(self, start, goal):
        steps = max(2, int(math.dist(start, goal) / 0.75) + 1)
        points = []
        for idx in range(steps):
            t = idx / max(1, steps - 1)
            points.append(
                (
                    start[0] + (goal[0] - start[0]) * t,
                    start[1] + (goal[1] - start[1]) * t,
                    self._clamp_z(start[2] + (goal[2] - start[2]) * t),
                )
            )
        return points

    def _densify_path(self, waypoints):
        points = []
        for a, b in zip(waypoints, waypoints[1:]):
            segment = self._sample_segment(a, b)
            if points:
                segment = segment[1:]
            points.extend(segment)
        return points

    def _corridor_point_from_cell(self, cell):
        x, y, _ = grid_to_world(self.spec, (cell[0], cell[1], 0))
        return (x, y, self._best_z_for_cell(cell, x, y))

    def _corridor_point(self, point):
        return (point[0], point[1], self._clamp_z(point[2]))

    def _best_z_for_cell(self, cell, x, y):
        if not self.adaptive_z_enabled:
            return self.corridor_default_z
        current_z = self.pose[2] if self.pose is not None else self.corridor_default_z
        scored = []
        for z in self.z_levels:
            if not self.is_point_in_aerial_free_space(x, y, z):
                continue
            iz = max(0, min(self.spec.z_cells - 1, int(z / self.spec.resolution)))
            gain = self._nearby_unknown_score((cell[0], cell[1], iz), radius=2)
            score = self.information_gain_z_weight * gain - self.prefer_current_altitude_weight * abs(z - current_z)
            scored.append((score, z))
        if not scored:
            return self.corridor_default_z
        z = max(scored)[1]
        if abs(z - self.corridor_default_z) > 0.05:
            self.adaptive_z_goal_count += 1
        return z

    def _best_z_for_xy(self, x, y):
        cell = world_to_grid(self.spec, (x, y, self.corridor_default_z))
        return self._best_z_for_cell(cell, x, y)

    def _clamp_z(self, z):
        return min(self.corridor_max_z, max(self.corridor_min_z, z))

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).lower() in ("1", "true", "yes", "on")

    def is_point_in_obstacle(self, x, y, z):
        idx = world_to_grid(self.spec, (x, y, z))
        return idx in self.occupied_voxels

    def is_point_in_aerial_free_space(self, x, y, z):
        if z < self.corridor_min_z or z > self.corridor_max_z:
            return False
        gx, gy, gz = world_to_grid(self.spec, (x, y, z))
        if not (0 <= gx < self.spec.x_cells and 0 <= gy < self.spec.y_cells):
            return False
        radius = int(math.ceil(self.corridor_clearance / self.spec.resolution))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if math.hypot(dx * self.spec.resolution, dy * self.spec.resolution) > self.corridor_clearance + self.spec.resolution * 0.5:
                    continue
                if (gx + dx, gy + dy, gz) in self.occupied_voxels:
                    return False
        return True

    def is_segment_collision_free_3d(self, a, b):
        steps = max(2, int(math.dist(a, b) / 0.25) + 1)
        for idx in range(steps):
            t = idx / max(1, steps - 1)
            p = (
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                self._clamp_z(a[2] + (b[2] - a[2]) * t),
            )
            if not self.is_point_in_aerial_free_space(*p):
                return False
        return True

    def _goal_pose(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
        msg.pose.orientation.w = 1.0
        return msg

    def _frontier_markers(self):
        arr = MarkerArray()
        marker = self._cube_list("frontiers", 7001, (0.0, 0.6, 1.0, 0.75), 0.35)
        for cell in self.map_state.get("frontier_cells", [])[:80]:
            p = self._corridor_point_from_cell(tuple(cell))
            if not self.is_point_in_aerial_free_space(*p):
                continue
            point = Point()
            point.x, point.y, point.z = p
            marker.points.append(point)
        arr.markers.append(marker)
        return arr

    def _viewpoint_markers(self):
        arr = MarkerArray()
        for idx, cell in enumerate(self.map_state.get("frontier_cells", [])[:12]):
            p = self._corridor_point_from_cell(tuple(cell))
            if not self.is_point_in_aerial_free_space(*p):
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "candidate_viewpoints"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = p
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.1, 1.0, 0.2, 0.82)
            arr.markers.append(marker)
        return arr

    def _goal_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "selected_goal"
        marker.id = 8001
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = goal
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.75
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 1.0)
        return marker

    def _start_pose_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "garage_start_pose"
        marker.id = 8101
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.start_pose[0]
        marker.pose.position.y = self.start_pose[1]
        marker.pose.position.z = self.start_pose[2]
        marker.pose.orientation.z = math.sin(self.start_pose[3] * 0.5)
        marker.pose.orientation.w = math.cos(self.start_pose[3] * 0.5)
        marker.scale.x = 0.85
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 0.95)
        return marker

    def _text_marker(self, ns, marker_id, text, xyz):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.45
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 1.0, 1.0, 1.0)
        marker.text = text
        return marker

    def _cube_list(self, ns, marker_id, color, scale):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker


def str_msg():
    from std_msgs.msg import String

    return String


def main(args=None):
    rclpy.init(args=args)
    node = AerialExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
