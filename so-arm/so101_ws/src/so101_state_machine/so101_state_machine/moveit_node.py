#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint,PositionConstraint, OrientationConstraint,BoundingVolume, PlanningScene, CollisionObject)
from moveit_msgs.msg import BoundingVolume
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
import numpy as np

class Arm_reach():
    def __init__(self, node: Node):
        
        self.node = node
        self._client = ActionClient(node, MoveGroup, '/move_action')
        self.node.get_logger().info('waiting for move_action server')
        self._client.wait_for_server()
        self.node.get_logger().info('connected! sending goal pose')
        self._scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
       
        self.is_moving = False
        self.last_result = None
        self._goal_start_time = None
        self._timeout_sec = 30.0

    def send_goal(self, x, y, z, r, p, yaw, orin=False, joint_constraints: dict = None):
        self.is_moving = True

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.9
        goal.request.max_acceleration_scaling_factor = 0.9
        goal.planning_options.plan_only = False

        c = Constraints()

   
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'gripper_frame_link'
        pc.weight = 1.0
        bv = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.015]
        bv.primitives.append(sp)
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        bv.primitive_poses.append(target_pose)
        pc.constraint_region = bv
        c.position_constraints.append(pc)

       
        if orin:
            oc = OrientationConstraint()
            oc.header.frame_id = 'base_link'
            oc.link_name = 'gripper_frame_link'
            oc.weight = 1.0
            q = quaternion_from_euler(r, p, yaw)
            oc.orientation.x = q[0]
            oc.orientation.y = q[1]
            oc.orientation.z = q[2]
            oc.orientation.w = q[3]
            oc.absolute_x_axis_tolerance = 0.4
            oc.absolute_y_axis_tolerance = 0.4
            oc.absolute_z_axis_tolerance = 0.4
            c.orientation_constraints.append(oc)

        
        if joint_constraints:
            for joint_name, position in joint_constraints.items():
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = position
                jc.tolerance_above = 0.2
                jc.tolerance_below = 0.2
                jc.weight = 0.3
                c.joint_constraints.append(jc)

        goal.request.goal_constraints.append(c)
        future = self._client.send_goal_async(goal)
        self._goal_start_time = time.monotonic()
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Arm goal rejected')
            self.is_moving = False
            self.last_result = False
            return
        self.node.get_logger().info('Arm goal accepted, executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.is_moving = False
        self._goal_start_time = None
        self.last_result = result.error_code.val == 1
        status = 'SUCCESS' if self.last_result else f'FAILED (code={result.error_code.val})'
        self.node.get_logger().info(f'Arm_reach: result {status}')

#----------------------------Bonus: Planning Scene Interface for adding collision objects and time out----------------------------#


    def is_timed_out(self) -> bool:
        if self._goal_start_time is None or not self.is_moving:
            return False
        return (time.monotonic() - self._goal_start_time) > self._timeout_sec
    
    def add_collision_box(self, name: str, x: float, y: float, z: float,sx: float, sy: float, sz: float, frame: str = 'base_link'):

        if not self._scene_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn('Planning scene service not available')
            return

        co = CollisionObject()
        co.id = name
        co.header.frame_id = frame
        co.operation = CollisionObject.ADD

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.BOX
        sp.dimensions = [sx, sy, sz]
        co.primitives.append(sp)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._scene_client.call_async(req)
        self.node.get_logger().info(f'colltion : added box [{name}]')

  
    def remove_collision_object(self, name: str):
        if not self._scene_client.wait_for_service(timeout_sec=2.0):
            return
        co = CollisionObject()
        co.id = name
        co.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.world.collision_objects.append(co)
        scene.is_diff = True

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self._scene_client.call_async(req)
        self.node.get_logger().info(f'Collision : removed [{name}]')

# def main():
#     rclpy.init()
#     node = Arm_reach()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
