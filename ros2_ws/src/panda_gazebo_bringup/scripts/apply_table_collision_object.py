#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class SceneCollisionObjectPublisher(Node):
    def __init__(self):
        super().__init__('scene_collision_object_publisher')

        self.declare_parameter('service_name', '/apply_planning_scene')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('object_id', 'table')
        self.declare_parameter('shape', 'box')
        self.declare_parameter('ignored_collision_link_names', '')
        self.declare_parameter('dimensions', [1.0, 1.0, 1.0])
        self.declare_parameter('position', [0.0, 0.0, 0.0])
        self.declare_parameter('orientation_rpy', [0.0, 0.0, 0.0])
        self.declare_parameter('get_scene_service_name', '/get_planning_scene')
        self.declare_parameter('initial_wait_sec', 0.0)
        self.declare_parameter('retry_period_sec', 1.0)
        self.declare_parameter('max_attempts', 20)

        self.service_name = (
            self.get_parameter('service_name').get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.object_id = self.get_parameter('object_id').get_parameter_value().string_value
        self.shape = self.get_parameter('shape').get_parameter_value().string_value.lower()
        ignored_collision_links_text = (
            self.get_parameter('ignored_collision_link_names')
            .get_parameter_value()
            .string_value
        )
        self.ignored_collision_link_names = [
            link_name.strip()
            for link_name in ignored_collision_links_text.split(',')
            if link_name.strip()
        ]
        self.dimensions = list(
            self.get_parameter('dimensions').get_parameter_value().double_array_value
        )
        self.position = list(
            self.get_parameter('position').get_parameter_value().double_array_value
        )
        self.orientation_rpy = list(
            self.get_parameter('orientation_rpy').get_parameter_value().double_array_value
        )
        self.get_scene_service_name = (
            self.get_parameter('get_scene_service_name').get_parameter_value().string_value
        )
        self.initial_wait_sec = (
            self.get_parameter('initial_wait_sec').get_parameter_value().double_value
        )
        self.retry_period_sec = (
            self.get_parameter('retry_period_sec').get_parameter_value().double_value
        )
        self.max_attempts = (
            self.get_parameter('max_attempts').get_parameter_value().integer_value
        )

        if self.shape not in {'box', 'cylinder'}:
            raise ValueError("shape must be either 'box' or 'cylinder'")
        expected_dimensions = 3 if self.shape == 'box' else 2
        if len(self.dimensions) != expected_dimensions:
            raise ValueError(
                f'dimensions must contain exactly {expected_dimensions} values for shape {self.shape}'
            )
        if len(self.position) != 3:
            raise ValueError('position must contain exactly 3 values: x, y, z')
        if len(self.orientation_rpy) != 3:
            raise ValueError('orientation_rpy must contain exactly 3 values: roll, pitch, yaw')
        if self.initial_wait_sec < 0.0:
            raise ValueError('initial_wait_sec must be greater than or equal to zero')
        if self.retry_period_sec <= 0.0:
            raise ValueError('retry_period_sec must be greater than zero')
        if self.max_attempts <= 0:
            raise ValueError('max_attempts must be greater than zero')

        self.apply_client = self.create_client(ApplyPlanningScene, self.service_name)
        self.get_scene_client = self.create_client(
            GetPlanningScene, self.get_scene_service_name
        )

    def wait_for_initial_delay(self):
        if self.initial_wait_sec <= 0.0:
            return
        self.get_logger().info(
            f'Waiting {self.initial_wait_sec:.1f}s before applying collision object {self.object_id}'
        )
        deadline = time.monotonic() + self.initial_wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_services(self):
        apply_ready = self.apply_client.wait_for_service(timeout_sec=1.0)
        get_scene_ready = self.get_scene_client.wait_for_service(timeout_sec=1.0)
        if apply_ready and get_scene_ready:
            return True
        self.get_logger().info(
            'Waiting for MoveIt planning scene services '
            f'{self.service_name} and {self.get_scene_service_name}'
        )
        return False

    def _build_planning_scene(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.id = self.object_id
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = (
            SolidPrimitive.BOX if self.shape == 'box' else SolidPrimitive.CYLINDER
        )
        primitive.dimensions = self.dimensions

        pose = Pose()
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self._quaternion_from_rpy(*self.orientation_rpy)
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_object)
        return scene

    def _copy_allowed_collision_matrix(self, matrix):
        copied_matrix = AllowedCollisionMatrix()
        copied_matrix.entry_names = list(matrix.entry_names)
        copied_matrix.default_entry_names = list(matrix.default_entry_names)
        copied_matrix.default_entry_values = list(matrix.default_entry_values)

        copied_entries = []
        for entry in matrix.entry_values:
            copied_entry = AllowedCollisionEntry()
            copied_entry.enabled = list(entry.enabled)
            copied_entries.append(copied_entry)
        copied_matrix.entry_values = copied_entries
        return copied_matrix

    @staticmethod
    def _normalize_allowed_collision_matrix(matrix):
        matrix_size = len(matrix.entry_names)

        while len(matrix.entry_values) < matrix_size:
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * matrix_size
            matrix.entry_values.append(entry)

        for entry in matrix.entry_values:
            enabled = list(entry.enabled)
            if len(enabled) < matrix_size:
                enabled.extend([False] * (matrix_size - len(enabled)))
            else:
                enabled = enabled[:matrix_size]
            entry.enabled = enabled

    def _ensure_allowed_collision_name(self, matrix, name):
        if name in matrix.entry_names:
            return

        matrix.entry_names.append(name)
        for entry in matrix.entry_values:
            entry.enabled = list(entry.enabled) + [False]

        new_entry = AllowedCollisionEntry()
        new_entry.enabled = [False] * len(matrix.entry_names)
        matrix.entry_values.append(new_entry)

    def _with_allowed_collision_exception(self, matrix, first_name, second_name):
        updated_matrix = self._copy_allowed_collision_matrix(matrix)
        self._normalize_allowed_collision_matrix(updated_matrix)
        self._ensure_allowed_collision_name(updated_matrix, first_name)
        self._ensure_allowed_collision_name(updated_matrix, second_name)
        self._normalize_allowed_collision_matrix(updated_matrix)

        first_index = updated_matrix.entry_names.index(first_name)
        second_index = updated_matrix.entry_names.index(second_name)
        updated_matrix.entry_values[first_index].enabled[second_index] = True
        updated_matrix.entry_values[second_index].enabled[first_index] = True
        return updated_matrix

    def _with_allowed_collision_exceptions(self, matrix):
        updated_matrix = self._copy_allowed_collision_matrix(matrix)
        for link_name in self.ignored_collision_link_names:
            updated_matrix = self._with_allowed_collision_exception(
                updated_matrix,
                link_name,
                self.object_id,
            )
        return updated_matrix

    @staticmethod
    def _quaternion_from_rpy(roll, pitch, yaw):
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        sin_roll = math.sin(half_roll)
        cos_roll = math.cos(half_roll)
        sin_pitch = math.sin(half_pitch)
        cos_pitch = math.cos(half_pitch)
        sin_yaw = math.sin(half_yaw)
        cos_yaw = math.cos(half_yaw)

        qx = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw
        qy = cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw
        qz = cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw
        qw = cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw
        return qx, qy, qz, qw

    def _spin_until_complete(self, future, timeout_sec):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        return future.result()

    def get_planning_scene_snapshot(self):
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        future = self.get_scene_client.call_async(request)
        try:
            response = self._spin_until_complete(future, timeout_sec=3.0)
        except Exception as error:  # pragma: no cover - transport/runtime error
            self.get_logger().warning(f'Failed to query the planning scene: {error}')
            return None

        if response is None:
            self.get_logger().warning('Timed out while querying the planning scene.')
            return None
        return response.scene

    def apply_collision_object(self, scene_snapshot):
        request = ApplyPlanningScene.Request()
        request.scene = self._build_planning_scene()
        if self.ignored_collision_link_names:
            request.scene.allowed_collision_matrix = self._with_allowed_collision_exceptions(
                scene_snapshot.allowed_collision_matrix
            )
        future = self.apply_client.call_async(request)
        try:
            response = self._spin_until_complete(future, timeout_sec=3.0)
        except Exception as error:  # pragma: no cover - transport/runtime error
            self.get_logger().warning(
                f'Failed to apply collision object {self.object_id}: {error}'
            )
            return False

        if response is None:
            self.get_logger().warning('Timed out while applying the planning scene update.')
            return False
        if not response.success:
            self.get_logger().warning('MoveIt rejected the planning scene update.')
            return False
        return True

    def collision_object_exists(self, scene_snapshot):
        return any(
            collision_object.id == self.object_id
            for collision_object in scene_snapshot.world.collision_objects
        )

    def collision_object_collisions_are_ignored(self, scene_snapshot):
        if not self.ignored_collision_link_names:
            return True

        matrix = scene_snapshot.allowed_collision_matrix
        try:
            object_index = matrix.entry_names.index(self.object_id)
        except ValueError:
            return False

        if object_index >= len(matrix.entry_values):
            return False

        object_row = list(matrix.entry_values[object_index].enabled)
        for link_name in self.ignored_collision_link_names:
            try:
                link_index = matrix.entry_names.index(link_name)
            except ValueError:
                return False

            if link_index >= len(matrix.entry_values):
                return False

            link_row = list(matrix.entry_values[link_index].enabled)
            if object_index >= len(link_row) or link_index >= len(object_row):
                return False
            if not (link_row[object_index] and object_row[link_index]):
                return False

        return True

    def run(self):
        self.wait_for_initial_delay()

        for attempt in range(1, self.max_attempts + 1):
            if not rclpy.ok():
                return

            if not self.wait_for_services():
                time.sleep(self.retry_period_sec)
                continue

            scene_snapshot = self.get_planning_scene_snapshot()
            if scene_snapshot is None:
                time.sleep(self.retry_period_sec)
                continue

            if self.collision_object_exists(
                scene_snapshot
            ) and self.collision_object_collisions_are_ignored(
                scene_snapshot
            ):
                self.get_logger().info(
                    f'Collision object {self.object_id} is already present in frame {self.frame_id}'
                )
                return

            self.get_logger().info(
                f'Applying collision object {self.object_id} (attempt {attempt}/{self.max_attempts})'
            )
            if not self.apply_collision_object(scene_snapshot):
                time.sleep(self.retry_period_sec)
                continue

            scene_snapshot = self.get_planning_scene_snapshot()
            if scene_snapshot is None:
                time.sleep(self.retry_period_sec)
                continue

            if self.collision_object_exists(
                scene_snapshot
            ) and self.collision_object_collisions_are_ignored(
                scene_snapshot
            ):
                self.get_logger().info(
                    f'Applied collision object {self.object_id} in frame {self.frame_id}'
                )
                return

            self.get_logger().warning(
                f'Collision object {self.object_id} is not fully configured in the planning scene yet.'
            )
            time.sleep(self.retry_period_sec)

        raise RuntimeError(
            f'Failed to verify collision object {self.object_id} after {self.max_attempts} attempts'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SceneCollisionObjectPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
