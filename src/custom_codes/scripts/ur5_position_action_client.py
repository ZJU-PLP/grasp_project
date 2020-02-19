#!/usr/bin/env python
# Borrowed and modified from the kinova-ros examples.

import rospy
import actionlib
# import kinova_msgs.msg
# import geometry_msgs.msg
# import std_msgs.msg
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf

# Inverse Kinematics Import
from ur_inverse_kinematics import *

"""
Calculate the initial robot position - Used before CPA application
Need to update: pass analytical homogeneous transformation to invKine
"""
def get_ik(position, orientation):
    matrix = tf.TransformerROS()

    # 0.15 is the offset from tool0 to grasping_link
    matrix2 = matrix.fromTranslationRotation((position[0]*(-1), position[1]*(-1), position[2] + 0.15),
                                             (orientation[0], orientation[1], orientation[2], orientation[3]))

    rospy.loginfo(matrix2)
    th = invKine(matrix2)
    sol1 = th[:, 2].transpose()
    joint_values_from_ik = np.array(sol1)

    joint_values = joint_values_from_ik[0, :]

    return joint_values.tolist()

def move_to_position(joint_values):
    """Send a cartesian goal to the action server."""
    global position_client
    if position_client is None:
        init()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                   'wrist_3_joint']
    # ordem do /joint_states -> elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_link, wrist_2_link, wrist_3_link

    goal.trajectory.points = [JointTrajectoryPoint(positions=joint_values, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),]
    position_client.send_goal(goal)

    if position_client.wait_for_result(rospy.Duration(10.0)):
        return position_client.get_result()
    else:
        position_client.cancel_all_goals()
        print('the cartesian action timed-out')
        return None

position_client = None

def init():
    global position_client
    
    position_client = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for server (pos_based_pos_traj_controller)..."
    position_client.wait_for_server()
    print "Connected to server (pos_based_pos_traj_controller)"
    
    # if args.realUR5:
    #     self.clientreal = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    #     print "Waiting for server (real)..."
    #     self.clientreal.wait_for_server()
    #     print "Connected to server (real)"
