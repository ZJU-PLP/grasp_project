#!/usr/bin/python

import rospy
import actionlib
import numpy as np
import argparse
import rosservice

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Header, ColorRGBA, Float32MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point, Vector3, Pose, TransformStamped, WrenchStamped

# Gazebo
from gazebo_msgs.msg import ModelState, ModelStates, ContactState, LinkState
from gazebo_msgs.srv import GetModelState, GetLinkState

from tf import TransformListener, TransformerROS, TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler, euler_from_matrix, quaternion_multiply

# import from moveit
from moveit_python import PlanningSceneInterface

# customized code
from get_geometric_jacobian import *
from ur_inverse_kinematics import *

#from pyquaternion import Quaternion

# tfBuffer = tf2_ros.Buffer()

MOVE_GRIPPER = True
CLOSE_GRIPPER_VEL = 0.05
OPEN_GRIPPER_VEL = -0.1
STOP_GRIPPER_VEL = 0.0
MIN_GRASP_ANGLE = 0.1
MAX_GRASP_ANGLE = 0.70
STARTED_GRIPPER = False
CONTACT = False
MAX_OPEN_INIT = 0.15
MAX_CLOSE_INIT = 0.10
GRIPPER_INIT = True
# GRASPING = True # False

def parse_args():
    parser = argparse.ArgumentParser(description='AAPF_Orientation')
    # store_false assumes that variable is already true and is only set to false if is given in command terminal
    parser.add_argument('--armarker', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    parser.add_argument('--gazebo', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    parser.add_argument('--dyntest', action='store_true', help='Follow dynamic goal from ar_track_alvar package')
    parser.add_argument('--OriON', action='store_true', help='Activate Orientation Control')
    args = parser.parse_args()
    return args

"""
Calculate the initial robot position - Used before CPA application
"""
def get_ik(pose):
    matrix = TransformerROS()
    # The orientation of /tool0 will be constant
    q = quaternion_from_euler(0, 3.14, 1.57)

    # The 0.15 accounts for the distance between tool0 and grasping link
    # The final height will be the set_distance (from base_link)
    offset = 0.15
    matrix2 = matrix.fromTranslationRotation((pose[0]*(-1), pose[1]*(-1), pose[2] + offset), (q[0], q[1], q[2], q[3]))
    th = invKine(matrix2)
    sol1 = th[:, 2].transpose()
    joint_values_from_ik = np.array(sol1)
    joint_values = joint_values_from_ik[0, :]
    return joint_values.tolist()

def turn_velocity_controller_on():
    rosservice.call_service('/controller_manager/switch_controller', [['joint_group_vel_controller'], ['pos_based_pos_traj_controller'], 1])

def turn_position_controller_on():
    rosservice.call_service('/controller_manager/switch_controller', [['pos_based_pos_traj_controller','gripper_controller'], ['joint_group_vel_controller'], 1])

class vel_control(object):
    def __init__(self, args, joint_values):
        self.args = args
        self.joint_values_home = joint_values

        # CPA PARAMETERS
        # At the end, the disciplacement will take place as a final orientation
        self.Displacement = [0.01, 0.01, 0.01]

        # CPA Parameters
        self.zeta = 0.5  # Attractive force gain of the goal
        self.max_error_allowed_pos_x = 0.010
        self.max_error_allowed_pos_y = 0.010
        self.max_error_allowed_pos_z = 0.006
        self.max_error_allowed_ori = 0.14
        self.dist_att = 0.1  # Influence distance in workspace
        self.dist_att_config = 0.2  # Influence distance in configuration space
        self.alfa_geral = 1.5 # multiply each alfa (position and rotation) equally
        self.gravity_compensation = 9
        self.alfa_pos = 4.5 * self.alfa_geral  # Grad step of positioning - Default: 0.5
        self.alfa_rot = 4 * self.alfa_geral  # Grad step of orientation - Default: 0.4

        # attributes used to receive msgs while publishing new ones
        self.processing = False
        self.new_msg = False
        self.msg = None

        # CPA Parameters
        self.diam_goal = 0.05

        # Topic used to publish vel commands
        self.pub_vel = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray,  queue_size=10)

        # Topic used to control the gripper
        # self.griper_pos = rospy.Publisher('/gripper/command', JointTrajectory,  queue_size=10)
        # self.gripper_msg = JointTrajectory()
        # self.gripper_msg.joint_names = ['robotiq_85_left_knuckle_joint']

        # self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_cmd", GripperCommandAction)
        # rospy.loginfo("Waiting for server (gripper_controller)...")
        # self.gripper_client.wait_for_server()
        # rospy.loginfo("Connected to server (gripper_controller)")

        self.joint_vels_gripper = Float64MultiArray()
        self.pub_vel_gripper = rospy.Publisher('/gripper_controller_vel/command', Float64MultiArray,  queue_size=10)

        # visual tools from moveit
        # self.scene = PlanningSceneInterface("base_link")
        self.marker_publisher = rospy.Publisher('visualization_marker2', Marker, queue_size=10)

        # Subscriber used to read joint values
        rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position, queue_size=10)

        # actionClient used to send joint positions
        self.client = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server (pos_based_pos_traj_controller)..."
        self.client.wait_for_server()
        print "Connected to server (pos_based_pos_traj_controller)"
        rospy.sleep(1)

        # Gazebo topics
        self.pub_model = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
        self.model = rospy.wait_for_message('gazebo/model_states', ModelStates)
        self.model_coordinates = rospy.ServiceProxy( '/gazebo/get_link_state', GetLinkState)
        self.wrench = rospy.Subscriber('/ft_sensor/raw', WrenchStamped, self.monitor_wrench, queue_size=10)
        # self.model_contacts = rospy.Subscriber('/gazebo/default/physics/contacts', ContactState, self.monitor_contacts, queue_size=10)

        # Standard attributes used to send joint position commands
        self.joint_vels = Float64MultiArray()
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory = JointTrajectory()
        self.goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                            'wrist_3_joint']
        self.initial_time = 4

        # Class attribute used to perform TF transformations
        self.tf = TransformListener()

        # Denavit-Hartenberg parameters of UR5
        # The order of the parameters is d1, SO, EO, a2, a3, d4, d45, d5, d6
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425, 0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15)
       

    """
    Adds spheres in RVIZ - Used to plot goals and obstacles
    """
    def add_sphere(self, pose, diam, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = 0
        marker.pose.position = Point(pose[0], pose[1], pose[2])
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale = Vector3(diam, diam, diam)
        marker.color = color
        self.marker_publisher.publish(marker)

    """
    Function to ensure safety
    """
    def safety_stop(self, ptAtual, wristPt):
        # High limit in meters of the end effector relative to the base_link
        high_limit = 0.01

        # Does not allow wrist_1_link to move above 20 cm relative to base_link
        high_limit_wrist_pt = 0.15

        if ptAtual[-1] < high_limit or wristPt[-1] < high_limit_wrist_pt:
            # Be careful. Only the limit of the end effector is being watched but the other
            # joint can also exceed this limit and need to be carefully watched by the operator
            rospy.loginfo("High limit of " + str(high_limit) + " exceeded!")
            self.home_pos()
            raw_input("\n==== Press enter to load Velocity Controller and start APF")
            turn_velocity_controller_on()

    """
    This method check if the goal position was reached
    """
    def all_close(self, goal, tolerance = 0.015):

        angles_difference = [self.actual_position[i] - goal[i] for i in range(6)]
        total_error = np.sum(angles_difference)

        if abs(total_error) > tolerance:
            return False

        return True

    """
    This method monitor the force applied to the gripper
    """       
    def monitor_wrench(self, msg):
        global MOVE_GRIPPER, STARTED_GRIPPER, CONTACT, GRASPING

        # print(msg)
        if STARTED_GRIPPER:
            if float(msg.wrench.force.x) < -2.0 or float(msg.wrench.force.x) > 2.0 or \
               float(msg.wrench.force.y) < -5.0 or float(msg.wrench.force.y) > 15.0 or \
               float(msg.wrench.force.z) < -4.0 or float(msg.wrench.force.z) > 5.0:
                MOVE_GRIPPER = False
                CONTACT = True

        # rate = rospy.Rate(60)
        # if GRASPING:

        #     object_coordinates_box = self.model_coordinates("robotiq_85_left_finger_tip_link", "base_link")
        #     z_position_box = object_coordinates_box.link_state.pose.position.z
        #     y_position_box = object_coordinates_box.link_state.pose.position.y
        #     x_position_box = object_coordinates_box.link_state.pose.position.x
                
        #     print "X, Y, Z: ", x_position_box, y_position_box, z_position_box

        #     obstacle = LinkState()
        #     obstacle.link_name = "box_link"
        #     obstacle.pose.position = Point(x_position_box, y_position_box, z_position_box)
        #     obstacle.reference_frame = "wrist_3_link"
        #     self.pub_model.publish(obstacle)
        #     rate.sleep()
            
    def gripper_init(self):
        global MOVE_GRIPPER
        global CLOSE_GRIPPER_VEL, OPEN_GRIPPER_VEL
        global MAX_OPEN_INIT, MAX_CLOSE_INIT

        if self.robotic > MAX_OPEN_INIT:
            gripper_vel = OPEN_GRIPPER_VEL
        else:
            gripper_vel = CLOSE_GRIPPER_VEL

        self.joint_vels_gripper.data = np.array([gripper_vel])
        self.pub_vel_gripper.publish(self.joint_vels_gripper)

        rate = rospy.Rate(125)
        rospy.loginfo("Starting the gripper! Please wait...")
        while not rospy.is_shutdown():
            rate.sleep()
            if gripper_vel == OPEN_GRIPPER_VEL:
                if self.robotic < MAX_CLOSE_INIT:
                    break
            else:
                if self.robotic > MAX_OPEN_INIT:
                    break

        self.joint_vels_gripper.data = np.array([0.0])
        self.pub_vel_gripper.publish(self.joint_vels_gripper)
        rospy.loginfo("Gripper started!")

    """
    Control the gripper by using velocity controller
    """
    def gripper_vel_control_close(self):
        global MOVE_GRIPPER

        rate = rospy.Rate(125)
        self.joint_vels_gripper.data = np.array([CLOSE_GRIPPER_VEL])
        self.pub_vel_gripper.publish(self.joint_vels_gripper)

        while not rospy.is_shutdown() and MOVE_GRIPPER and self.robotic < 0.6:
            rate.sleep()

        # stops the robot after the goal is reached
        rospy.loginfo("Gripper stopped!")
        print("Angle: ", self.robotic)
        self.joint_vels_gripper.data = np.array([0.0])
        self.pub_vel_gripper.publish(self.joint_vels_gripper)


    """
    Control the gripper by using velocity controller
    """
    def gripper_vel_control_open(self):
        global MOVE_GRIPPER

        rate = rospy.Rate(125)
        self.joint_vels_gripper.data = np.array([OPEN_GRIPPER_VEL])
        self.pub_vel_gripper.publish(self.joint_vels_gripper)
        
        while not rospy.is_shutdown() and MOVE_GRIPPER and self.robotic > 0.1:
            rate.sleep()
           
        # stops the robot after the goal is reached
        rospy.loginfo("Gripper stopped!")
        self.joint_vels_gripper.data = np.array([0.0])
        self.pub_vel_gripper.publish(self.joint_vels_gripper)

    """
    The joint states published by /joint_staes of the UR5 robot are in wrong order.
    /joint_states topic normally publishes the joint in the following order:
    [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    But the correct order of the joints that must be sent to the robot is:
    ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    """
    def ur5_actual_position(self, joint_values_from_ur5):
        if self.args.gazebo:
            self.th3, self.robotic, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
            # print("Robotic angle: ", self.robotic)
        else:
            self.th3, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
        self.actual_position = [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6]

  
    """
    Send the HOME position to the robot
    self.client.wait_for_result() won't work well.
    Instead, a while loop has been created to ensure that the robot reaches the
    goal even after the failure.
    In order to avoid the gripper to keep moving after the node is killed, the method gripper_init() is also called
    """
    def home_pos(self):
        global GRIPPER_INIT

        turn_position_controller_on()
        rospy.sleep(0.1)

        if GRIPPER_INIT:
            self.gripper_init()

        # First point is current position
        try:
            self.goal.trajectory.points = [(JointTrajectoryPoint(positions=self.joint_values_home, velocities=[0]*6, time_from_start=rospy.Duration(self.initial_time)))]
            self.initial_time += 1
            if not self.all_close(self.joint_values_home):
                print "'Homing' the robot."
                self.client.send_goal(self.goal)
                self.client.wait_for_result()
                while not self.all_close(self.joint_values_home):
                    self.client.send_goal(self.goal)
                    self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

        print "\n==== Goal reached!"

    def move_to_pos(self, joint_values):
        try:
            self.goal.trajectory.points = [(JointTrajectoryPoint(positions=joint_values, velocities=[0]*6, time_from_start=rospy.Duration(18.0)))]
            if not self.all_close(joint_values):
                # raw_input("==== Press enter to move the robot to the grasp position!")
                # print "Moving the robot."
                self.client.send_goal(self.goal)
                self.client.wait_for_result()
                while not self.all_close(joint_values):
                    self.client.send_goal(self.goal)
                    self.client.wait_for_result()
        except KeyboardInterrupt:
            self.client.cancel_goal()
            raise
        except:
            raise

def main():
    global MOVE_GRIPPER, STARTED_GRIPPER, GRIPPER_INIT, GRASPING

    arg = parse_args()

    # Turn position controller ON
    turn_position_controller_on()

    # Calculate joint values equivalent to the HOME position
    joint_values = get_ik([-0.4, -0.10, 0.35])
    
    ur5_vel = vel_control(arg, joint_values)
    

    # Stop the robot in case of the node is killed
    rospy.on_shutdown(ur5_vel.home_pos)

    # Send the robot to the custom HOME position
    
    raw_input("==== Press enter to 'home' the robot and open gripper!")
    ur5_vel.home_pos()
    ur5_vel.gripper_init()
    GRIPPER_INIT = False

    msg = rospy.wait_for_message('/ggcnn/out/command', Float32MultiArray)
    ur5_vel.tf.waitForTransform("base_link", "object_detected", rospy.Time(), rospy.Duration(4.0))
    d = list(msg.data)
    posCB, _ = ur5_vel.tf.lookupTransform("base_link", "object_detected", rospy.Time())
    _, oriObjCam = ur5_vel.tf.lookupTransform("camera_depth_optical_frame", "object_detected", rospy.Time())
    ori = euler_from_quaternion(oriObjCam)
    
    # gripper_pose = int(np.clip((13.-230.)/0.087 * d[-1] + 230., 0, 255))
    # percent = 0.087 * pose / 255

    # print(posCB)

    ur5_vel.add_sphere([posCB[0], posCB[1], posCB[2]], 0.05, ColorRGBA(0.0, 1.0, 0.0, 1.0))
    joint_values = get_ik([posCB[0], posCB[1], posCB[2]])
    # print("Ori: ", ori[-1] * 180 / np.pi)
    joint_values[-1] = ori[-1] 
    
    raw_input("==== Press enter to move the robot to the goal position!")
    ur5_vel.move_to_pos(joint_values)

    # Start monitoring gripper torques
    STARTED_GRIPPER = True

    raw_input("==== Press enter to close the gripper!")
    # GRASPING = True
    ur5_vel.gripper_vel_control_close()    

    # After this raw_input, the rospy.on_shutdown will be called
    raw_input("==== Press enter to 'home' the robot!")
    ur5_vel.home_pos()

    raw_input("==== Press enter to open the gripper!")
    MOVE_GRIPPER = True    
    ur5_vel.gripper_vel_control_open()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
	    print "Program interrupted before completion"
