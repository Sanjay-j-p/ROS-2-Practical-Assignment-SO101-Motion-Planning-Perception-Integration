#!/usr/bin/env python3
"""
INTERVIEW TEMPLATE: ROS2 + py_trees Behaviour Tree (Implementation-Agnostic)

Task sequence:
1) open_gripper
2) grabbing
3) attach (simulation)        <-- PROVIDED (from our side)
4) move_to_box_position
5) detach (simulation)        <-- PROVIDED (from our side)
6) open_gripper

Instructions:
- Implement ONLY the TODO logic in OpenGripper / Grabbing / MoveToBoxPosition.
- You can use ANY ROS approach you want (services/actions/topics/MoveIt/etc.).
- Each leaf must return proper py_trees status: RUNNING / SUCCESS / FAILURE.
- Do not block inside update() (no sleep). Use state/futures/timers if needed.
"""

#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import py_trees
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from so101_state_machine.moveit_node import Arm_reach

# -------------------------
# Candidate BT Leaves (blank)
# -------------------------
class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        
        # TODO: initialise any clients/publishers/actions you want here
        # self._gripper_done = False
        self._start_time=None
        self.pub=self.node.create_publisher(JointTrajectory,'/gripper_controller/joint_trajectory',10)

        
        

    def initialise(self):
        self._gripper_done = False
        msg=JointTrajectory()
        msg.joint_names=['gripper']
        point=JointTrajectoryPoint()
        point.time_from_start=Duration(sec=1)
        point.positions=[0.7]
        msg.points.append(point)
        self.pub.publish(msg)
        self._start_time = time.monotonic()
        # self.node.create_timer(1.2, self._set_done) 
    

    def update(self) -> py_trees.common.Status:
        # TODO: implement open gripper
        # Return RUNNING until done, then SUCCESS/FAILURE
        if (time.monotonic() - self._start_time) < 2.5:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS

class Grabbing(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        self.override = False  # optional: for handling timeouts in BT (keeps it simple)

        self.node.create_subscription(PoseStamped, '/detected_cup_pose', self._cup_cb, 10)
        self.node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        self.arm_move = Arm_reach(self.node)

        self.arm_move.add_collision_box(name='target_box',x=0.2335, y=0.0, z=0.08,sx=0.15, sy=0.05, sz=0.16)

        self.arm_pub = self.node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.node.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.cup_pose = None
        self.joints = None
        self.phase = 0
        self.timer = None

    def _cup_cb(self, msg: PoseStamped):
        self.cup_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def _joint_cb(self, msg: JointState):
        self.joints = dict(zip(msg.name, msg.position))

    def initialise(self):
        self.phase = 0
        self.timer = None
        self.arm_move.last_result = None

    def update(self) -> py_trees.common.Status:

        if self.cup_pose is None or self.joints is None:
            return py_trees.common.Status.RUNNING
        if self.phase == 0:
            self.arm_move.send_goal(self.cup_pose[0], self.cup_pose[1],self.cup_pose[2], 0, 1.2, 0)
            self.phase = 1
            return py_trees.common.Status.RUNNING
        if self.phase == 1:
            if self.arm_move.is_timed_out() and self.override:
                self.node.get_logger().error('Grabbing: arm execution timed out')
                self.arm_move.is_moving = False
                return py_trees.common.Status.FAILURE
            if self.arm_move.is_moving or self.arm_move.last_result is None:
                return py_trees.common.Status.RUNNING
            if not self.arm_move.last_result:
                return py_trees.common.Status.FAILURE
            self._send_gripper(delay_sec=2)
            self.gripper_start = time.monotonic()
            self.phase = 2
            return py_trees.common.Status.RUNNING
        if self.phase == 2:
            if (time.monotonic() - self.gripper_start) < 2.5:
                return py_trees.common.Status.RUNNING
            self._send_wrist(delay_sec=2)
            self.wrist_start = time.monotonic()
            self.phase = 3
            return py_trees.common.Status.RUNNING
        if self.phase == 3:
            if (time.monotonic() - self.wrist_start) < 4.5:
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


    def _send_wrist(self, delay_sec: int):
        msg = JointTrajectory()
        msg.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=delay_sec)
        point.positions = [
            self.joints.get('shoulder_pan', 0.0),
            self.joints.get('shoulder_lift', 0.0),
            self.joints.get('elbow_flex', 0.0),
            0.0, 
            0.0  
        ]
        point.velocities = [0.0] * 5
        msg.points.append(point)
        # self.arm_pub.publish(msg)  #  Optional : publish to arm controller for wrist adjustment (added due to not reaching teh goal)

    
    def _send_gripper(self, delay_sec: int):
        msg = JointTrajectory()
        msg.joint_names = ['gripper']

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(sec=delay_sec)
        point.positions = [0.0]  
        msg.points.append(point)
        self.gripper_pub.publish(msg)

class MoveToBoxPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, node: Node):
        super().__init__(name)
        self.node = node
        # TODO: initialise any clients/publishers/actions 

        self.arm_move=Arm_reach(self.node)
        self.override = False  # optional: for handling timeouts in BT (keeps it simple)
        self.arm_move.last_result = None


        # self.node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        # self.pub=self.node.create_publisher(JointTrajectory,'/gripper_controller/joint_trajectory',10)
        
    # def _joint_cb(self, msg: JointState):
    #     self.joints = dict(zip(msg.name, msg.position))

    def initialise(self):
        # TODO: reset internal state

        self.arm_move.last_result = None
        self.arm_move.is_moving = False
        self.arm_move.send_goal(0.2335, -0.127, 0.2, 2.5158, -0.6102, 0.0274,orin=True)
        self.node.get_logger().info('Moving to box position...')

    def update(self) -> py_trees.common.Status:
        # TODO: implement move to box position
        
        if self.arm_move.is_timed_out() and self.override:
            self.node.get_logger().error('Grabbing: arm execution timed out')
            self.arm_move.is_moving = False
            return py_trees.common.Status.FAILURE
        
        if self.arm_move.is_moving:
            return py_trees.common.Status.RUNNING
        if self.arm_move.last_result is None:
            return py_trees.common.Status.RUNNING  
        if self.arm_move.last_result:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


# -------------------------
# PROVIDED: Attach / Detach Cube BT Leaf 
# -------------------------
class AttachDetachCube(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, topic_name, attach, delay_sec=1.0):
        super().__init__(name)
        self.node = node
        self.topic_name = topic_name
        self.attach = attach
        self.delay_sec = delay_sec

        self.pub = self.node.create_publisher(Bool, topic_name, 10)
        self._start_time = None
        self._done = False

    def initialise(self):
        self._start_time = time.monotonic()
        self._done = False

    def update(self):
        if not self._done and (time.monotonic() - self._start_time) >= self.delay_sec:
            msg = Bool()
            msg.data = self.attach
            self.pub.publish(msg)
            self.node.get_logger().info(
                f"BT: Isaac attach={self.attach} on {self.topic_name}"
            )
            self._done = True
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING


# -------------------------
# Tree
# -------------------------
def create_tree(node: Node):
    STEP_RETRIES = 2  # optional retry per-step (kept simple)
    ATTACH_TOPIC = "/isaac_attach_cube"
    ATTACH_DELAY = 0.5

    seq = py_trees.composites.Sequence(name="TaskSequence", memory=True)

    seq.add_children([
        py_trees.decorators.Retry(
            "RetryOpen1",
            OpenGripper("OpenGripper1", node),
            STEP_RETRIES,
        ),
        py_trees.decorators.Retry(
            "RetryGrabbing",
            Grabbing("Grabbing", node),
            STEP_RETRIES,
        ),

        # PROVIDED
        AttachDetachCube("AttachCube", node, ATTACH_TOPIC, attach=True, delay_sec=ATTACH_DELAY),

        py_trees.decorators.Retry(
            "RetryMoveToBox",
            MoveToBoxPosition("MoveToBoxPosition", node),
            STEP_RETRIES,
        ),

        # PROVIDED
        AttachDetachCube("DetachCube", node, ATTACH_TOPIC, attach=False, delay_sec=ATTACH_DELAY),

        py_trees.decorators.Retry(
            "RetryOpen2",
            OpenGripper("OpenGripper2", node),
            STEP_RETRIES,
        ),
    ])

    # Run once (optional, keeps it from repeating forever)
    root = py_trees.decorators.OneShot(
        name="RunOnce",
        child=seq,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION
    )

    return root


# -------------------------
# Node
# -------------------------
class BTNode(Node):
    def __init__(self):
        super().__init__("bt_interview_template_node")

        self.tree = py_trees.trees.BehaviourTree(create_tree(self))
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info("Interview BT template node started.")

    def _tick(self):
        self.tree.tick()


def main():
    rclpy.init()
    node = BTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
