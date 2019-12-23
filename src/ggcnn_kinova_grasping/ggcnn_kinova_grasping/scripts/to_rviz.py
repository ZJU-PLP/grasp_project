#!/usr/bin/env python
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, pose):

    
    # br = tf.TransformBroadcaster()
    # br.sendTransform((msg.x, msg.y, 0),
    #                  tf.transformations.quaternion_from_euler(0, 0, msg.theta),
    #                  rospy.Time.now(),
    #                  turtlename,
    #                  "world")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/ggcnn/out/command', Float32MultiArray, handle_turtle_pose, queue_size=1)
    rospy.spin()
