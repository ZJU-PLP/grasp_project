#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import tf
import turtlesim.msg

def handle_turtle_pose(msg):
    # print(msg.data[0])
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.data[0]/1000, msg.data[1]/1000, msg.data[2]/1000),
                     tf.transformations.quaternion_from_euler(0, 0, msg.data[3]),
                     rospy.Time.now(),
                     "link_novo",
                     "kinect2_link")

if __name__ == '__main__':
    rospy.init_node('goal_tf_broadcaster')
    rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, handle_turtle_pose, queue_size=1)
    rospy.spin()
