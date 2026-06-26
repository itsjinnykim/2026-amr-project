#!/usr/bin/env python3
"""Live Nav2 score monitor for Storagy score-based navigation tests.

Subscribes to common Nav2 topics and prints/publishes a compact score:

  score = alpha * path_length
        + beta  * elapsed_time
        + gamma * accumulated_abs_rotation
        + delta * safety_penalty
        + energy_weight * estimated_energy_cost
        + blocked_penalty

The node is intentionally lightweight and uses only standard ROS 2 messages.
It is a measurement tool, not a controller.
"""

from __future__ import annotations

import csv
import math
import os
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class ScoreState:
    path_length_m: float = 0.0
    nearest_obstacle_m: Optional[float] = None
    inflated_cells: int = 0
    high_cost_cells: int = 0
    mean_cost: float = 0.0
    accumulated_rotation_rad: float = 0.0
    accumulated_velocity_change: float = 0.0
    accumulated_acceleration_change: float = 0.0
    stop_count: int = 0
    restart_count: int = 0
    last_cmd_time_s: Optional[float] = None
    last_report_time_s: Optional[float] = None
    low_motion_start_s: Optional[float] = None
    last_linear_x: Optional[float] = None
    last_angular_z: Optional[float] = None
    last_linear_accel: Optional[float] = None
    last_angular_accel: Optional[float] = None
    was_low_motion: Optional[bool] = None
    blocked: bool = False


class Nav2ScoreMonitor(Node):
    def __init__(self) -> None:
        super().__init__("nav2_score_monitor")

        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("report_topic", "/nav2_score_report")
        self.declare_parameter("csv_path", "")

        self.declare_parameter("distance_weight", 1.0)
        self.declare_parameter("time_weight", 0.05)
        self.declare_parameter("rotation_weight", 0.5)
        self.declare_parameter("safety_weight", 4.0)
        self.declare_parameter("energy_weight", 0.8)
        self.declare_parameter("blocked_weight", 8.0)
        self.declare_parameter("velocity_change_weight", 1.0)
        self.declare_parameter("acceleration_change_weight", 0.2)
        self.declare_parameter("stop_weight", 1.0)
        self.declare_parameter("restart_weight", 1.0)

        self.declare_parameter("safety_distance_m", 0.70)
        self.declare_parameter("blocked_obstacle_distance_m", 0.45)
        self.declare_parameter("blocked_seconds", 3.0)
        self.declare_parameter("linear_motion_threshold", 0.03)
        self.declare_parameter("angular_motion_threshold", 0.05)
        self.declare_parameter("report_period_s", 1.0)

        self.state = ScoreState()
        self.start_time_s = self._now_s()
        self.last_twist = Twist()

        self.report_pub = self.create_publisher(
            String, self.get_parameter("report_topic").value, 10
        )
        self.create_subscription(
            Path, self.get_parameter("path_topic").value, self._on_path, 10
        )
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self._on_cmd_vel, 20
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("costmap_topic").value,
            self._on_costmap,
            5,
        )
        self.timer = self.create_timer(
            float(self.get_parameter("report_period_s").value), self._report
        )

        self.csv_file = None
        self.csv_writer = None
        csv_path = str(self.get_parameter("csv_path").value)
        if csv_path:
            os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
            self.csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.DictWriter(
                self.csv_file,
                fieldnames=[
                    "time_s",
                    "score",
                    "path_length_m",
                    "nearest_obstacle_m",
                    "inflated_cells",
                    "high_cost_cells",
                    "mean_cost",
                    "accumulated_rotation_rad",
                    "accumulated_velocity_change",
                    "accumulated_acceleration_change",
                    "stop_count",
                    "restart_count",
                    "estimated_energy_cost",
                    "safety_penalty",
                    "blocked",
                    "linear_x",
                    "angular_z",
                ],
            )
            self.csv_writer.writeheader()

        self.get_logger().info("Nav2 score monitor started")

    def destroy_node(self) -> bool:
        if self.csv_file:
            self.csv_file.close()
        return super().destroy_node()

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_path(self, msg: Path) -> None:
        length = 0.0
        poses = msg.poses
        for prev, cur in zip(poses, poses[1:]):
            dx = cur.pose.position.x - prev.pose.position.x
            dy = cur.pose.position.y - prev.pose.position.y
            length += math.hypot(dx, dy)
        self.state.path_length_m = length

    def _on_cmd_vel(self, msg: Twist) -> None:
        now = self._now_s()
        if self.state.last_cmd_time_s is not None:
            dt = max(0.0, now - self.state.last_cmd_time_s)
            self.state.accumulated_rotation_rad += abs(msg.angular.z) * dt
            self._update_energy_motion(msg, dt)
        else:
            self._update_low_motion_transition(msg)

        self.state.last_cmd_time_s = now
        self.state.last_linear_x = msg.linear.x
        self.state.last_angular_z = msg.angular.z
        self.last_twist = msg
        self._update_blocked_state(now, msg)

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        if resolution <= 0.0 or width == 0 or height == 0:
            return

        center_x = width * 0.5
        center_y = height * 0.5
        nearest = None
        inflated_cells = 0
        high_cost_cells = 0
        cost_sum = 0.0
        cost_count = 0

        for index, value in enumerate(msg.data):
            if value < 0:
                continue
            cost_sum += value
            cost_count += 1
            if 0 < value < 80:
                inflated_cells += 1
            elif value >= 80:
                high_cost_cells += 1

            if value < 80:
                continue
            x = index % width
            y = index // width
            dist = math.hypot(x - center_x, y - center_y) * resolution
            if nearest is None or dist < nearest:
                nearest = dist

        self.state.nearest_obstacle_m = nearest
        self.state.inflated_cells = inflated_cells
        self.state.high_cost_cells = high_cost_cells
        self.state.mean_cost = cost_sum / cost_count if cost_count else 0.0

    def _update_blocked_state(self, now: float, msg: Twist) -> None:
        linear_threshold = float(self.get_parameter("linear_motion_threshold").value)
        angular_threshold = float(self.get_parameter("angular_motion_threshold").value)
        blocked_obstacle_distance = float(
            self.get_parameter("blocked_obstacle_distance_m").value
        )
        blocked_seconds = float(self.get_parameter("blocked_seconds").value)

        near_obstacle = (
            self.state.nearest_obstacle_m is not None
            and self.state.nearest_obstacle_m <= blocked_obstacle_distance
        )
        low_motion = (
            abs(msg.linear.x) < linear_threshold
            and abs(msg.angular.z) < angular_threshold
        )
        goal_still_far = self.state.path_length_m > 0.5

        if near_obstacle and low_motion and goal_still_far:
            if self.state.low_motion_start_s is None:
                self.state.low_motion_start_s = now
            self.state.blocked = (now - self.state.low_motion_start_s) >= blocked_seconds
        else:
            self.state.low_motion_start_s = None
            self.state.blocked = False

    def _update_energy_motion(self, msg: Twist, dt: float) -> None:
        if dt <= 0.0:
            self._update_low_motion_transition(msg)
            return

        last_linear = self.state.last_linear_x
        last_angular = self.state.last_angular_z
        if last_linear is None or last_angular is None:
            self._update_low_motion_transition(msg)
            return

        linear_delta = msg.linear.x - last_linear
        angular_delta = msg.angular.z - last_angular
        self.state.accumulated_velocity_change += (
            abs(linear_delta) + abs(angular_delta)
        )

        linear_accel = linear_delta / dt
        angular_accel = angular_delta / dt
        if self.state.last_linear_accel is not None:
            self.state.accumulated_acceleration_change += abs(
                linear_accel - self.state.last_linear_accel
            )
        if self.state.last_angular_accel is not None:
            self.state.accumulated_acceleration_change += abs(
                angular_accel - self.state.last_angular_accel
            )
        self.state.last_linear_accel = linear_accel
        self.state.last_angular_accel = angular_accel
        self._update_low_motion_transition(msg)

    def _update_low_motion_transition(self, msg: Twist) -> None:
        low_motion = self._is_low_motion(msg)
        was_low_motion = self.state.was_low_motion
        if was_low_motion is False and low_motion:
            self.state.stop_count += 1
        elif was_low_motion is True and not low_motion:
            self.state.restart_count += 1
        self.state.was_low_motion = low_motion

    def _is_low_motion(self, msg: Twist) -> bool:
        linear_threshold = float(self.get_parameter("linear_motion_threshold").value)
        angular_threshold = float(self.get_parameter("angular_motion_threshold").value)
        return (
            abs(msg.linear.x) < linear_threshold
            and abs(msg.angular.z) < angular_threshold
        )

    def _safety_penalty(self) -> float:
        safety_distance = float(self.get_parameter("safety_distance_m").value)
        nearest = self.state.nearest_obstacle_m
        if nearest is None:
            return 0.0
        return max(0.0, safety_distance - nearest)

    def _energy_cost(self) -> float:
        velocity_weight = float(self.get_parameter("velocity_change_weight").value)
        accel_weight = float(self.get_parameter("acceleration_change_weight").value)
        stop_weight = float(self.get_parameter("stop_weight").value)
        restart_weight = float(self.get_parameter("restart_weight").value)
        return (
            velocity_weight * self.state.accumulated_velocity_change
            + accel_weight * self.state.accumulated_acceleration_change
            + stop_weight * self.state.stop_count
            + restart_weight * self.state.restart_count
        )

    def _score(self, elapsed_s: float) -> float:
        alpha = float(self.get_parameter("distance_weight").value)
        beta = float(self.get_parameter("time_weight").value)
        gamma = float(self.get_parameter("rotation_weight").value)
        delta = float(self.get_parameter("safety_weight").value)
        energy_weight = float(self.get_parameter("energy_weight").value)
        blocked_weight = float(self.get_parameter("blocked_weight").value)
        return (
            alpha * self.state.path_length_m
            + beta * elapsed_s
            + gamma * self.state.accumulated_rotation_rad
            + delta * self._safety_penalty()
            + energy_weight * self._energy_cost()
            + (blocked_weight if self.state.blocked else 0.0)
        )

    def _report(self) -> None:
        elapsed = self._now_s() - self.start_time_s
        score = self._score(elapsed)
        energy_cost = self._energy_cost()
        nearest = self.state.nearest_obstacle_m
        nearest_text = "none" if nearest is None else f"{nearest:.2f}"
        line = (
            f"score={score:.3f}, path={self.state.path_length_m:.2f}m, "
            f"obstacle={nearest_text}m, rotation={self.state.accumulated_rotation_rad:.2f}rad, "
            f"energy={energy_cost:.2f}, stops={self.state.stop_count}, "
            f"restarts={self.state.restart_count}, "
            f"safety_penalty={self._safety_penalty():.2f}, "
            f"inflated={self.state.inflated_cells}, high_cost={self.state.high_cost_cells}, "
            f"blocked={self.state.blocked}"
        )
        self.get_logger().info(line)
        self.report_pub.publish(String(data=line))

        if self.csv_writer:
            self.csv_writer.writerow(
                {
                    "time_s": f"{elapsed:.3f}",
                    "score": f"{score:.6f}",
                    "path_length_m": f"{self.state.path_length_m:.6f}",
                    "nearest_obstacle_m": "" if nearest is None else f"{nearest:.6f}",
                    "inflated_cells": self.state.inflated_cells,
                    "high_cost_cells": self.state.high_cost_cells,
                    "mean_cost": f"{self.state.mean_cost:.6f}",
                    "accumulated_rotation_rad": f"{self.state.accumulated_rotation_rad:.6f}",
                    "accumulated_velocity_change": f"{self.state.accumulated_velocity_change:.6f}",
                    "accumulated_acceleration_change": f"{self.state.accumulated_acceleration_change:.6f}",
                    "stop_count": self.state.stop_count,
                    "restart_count": self.state.restart_count,
                    "estimated_energy_cost": f"{energy_cost:.6f}",
                    "safety_penalty": f"{self._safety_penalty():.6f}",
                    "blocked": str(self.state.blocked).lower(),
                    "linear_x": f"{self.last_twist.linear.x:.6f}",
                    "angular_z": f"{self.last_twist.angular.z:.6f}",
                }
            )
            self.csv_file.flush()


def main() -> None:
    rclpy.init()
    node = Nav2ScoreMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
