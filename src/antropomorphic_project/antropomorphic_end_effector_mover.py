#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from antropomorphic_project.msg import EndEffector
from antropomorphic_project.move_joints import JointMover
from antropomorphic_project.rviz_marker import MarkerBasics
from antropomorphic_project.ik_antropomorphic_arm import AntroArmIK, EndEffectorWorkingSpace3D

class AntroEEMover(object):
    def __init__(self):
        rospy.loginfo("Initializing AntroEEMover...")
        self._rate = rospy.Rate(20.0)
        
        # Instance to move joints
        self.joint_mover = JointMover()
        # Instance to visualize goal points
        self.marker = MarkerBasics()

        # Initialize IK solver
        DH_parameters = {
            "r1": 0.0,
            "r2": 1.0,
            "r3": 1.0
        }
        self.ik_solver = AntroArmIK(DH_parameters)

        self.last_goal_pose = None
        self.pose_queue = []

        self.ee_cmd_sub = rospy.Subscriber(
            "/ee_pose_commands", EndEffector, self.ee_pose_callback, queue_size=1
        )
        self.ee_real_pose_sub = rospy.Subscriber(
            "/end_effector_real_pose", Vector3, self.ee_real_pose_callback, queue_size=1
        )

    def ee_pose_callback(self, ee_msg):
        self.last_goal_pose = ee_msg.ee_xy_theta

        # Add the received pose to the queue
        self.pose_queue.append(ee_msg)

    def ee_real_pose_callback(self, msg):
        if self.last_goal_pose is not None:
            error_x = self.last_goal_pose.x - msg.x
            error_y = self.last_goal_pose.y - msg.y
            error_z = self.last_goal_pose.z - msg.z
            rospy.loginfo(
                f"EE Real Pose: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) | "
                f"Goal: ({self.last_goal_pose.x:.3f}, {self.last_goal_pose.y:.3f}, {self.last_goal_pose.z:.3f}) | "
                f"Error: ({error_x:.3f}, {error_y:.3f}, {error_z:.3f})"
            )

    def execute(self):
        while not rospy.is_shutdown():
            if self.pose_queue:
                ee_msg = self.pose_queue.pop(0)
                elbow_policy = ee_msg.elbow_policy.data
                self.last_goal_pose = ee_msg.ee_xy_theta
                goal_x = ee_msg.ee_xy_theta.x
                goal_y = ee_msg.ee_xy_theta.y
                goal_z = ee_msg.ee_xy_theta.z

                # Publish goal point for visualization
                self.marker.publish_point(goal_x, goal_y, goal_z)

                # Use IK solver to get joint angles
                ee_pose = EndEffectorWorkingSpace3D(Px_ee=goal_x, Py_ee=goal_y, Pz_ee=goal_z)
                elbow_config = "down" if elbow_policy == "plus-minus" else "up"
                theta_array, solution_possible = self.ik_solver.compute_ik(ee_pose, elbow_config=elbow_config)

                if solution_possible and len(theta_array) == 3:
                    theta_1, theta_2, theta_3 = theta_array
                    self.joint_mover.move_all_joints(theta_1, theta_2, theta_3)
                else:
                    rospy.logwarn("No IK solution found for the given end-effector pose.")
            self._rate.sleep()

if __name__ == "__main__":
    rospy.init_node("antropomorphic_end_effector_mover")
    arm_mover = AntroEEMover()

    try:
        arm_mover.execute()
    except rospy.ROSInterruptException:
        pass

