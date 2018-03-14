#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

pub = rospy.Publisher('/odometry/corrected', Odometry, queue_size=30)
odom_corrected = Odometry()
transtamp = TransformStamped()
tf_new = TransformStamped()
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
br = tf2_ros.TransformBroadcaster()

def odometryCb(data):
    odom_corrected = data
    odom_corrected.header.frame_id = "odom_corrected"
    odom_corrected.child_frame_id = "base_link"
    odom_corrected.pose.pose.position.z = 0
    pub.publish(odom_corrected)

def timer_callback(event):
    try:
        transtamp = tf_buffer.lookup_transform('odom','base_link',rospy.Time())
    except tf2_ros.LookupException:
        rospy.loginfo("Missed")
        pass

    z_offset = transtamp.transform.translation.z

    tf_new.header.stamp = rospy.Time.now()
    tf_new.header.frame_id = "odom_corrected"
    tf_new.child_frame_id = "odom"
    tf_new.transform.translation.x = 0
    tf_new.transform.translation.y = 0
    tf_new.transform.translation.z = -z_offset
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    tf_new.transform.rotation.x = q[0]
    tf_new.transform.rotation.y = q[1]
    tf_new.transform.rotation.z = q[2]
    tf_new.transform.rotation.w = q[3]

    br.sendTransform(tf_new)

if __name__ == "__main__":
    rospy.init_node('z_correction') #make node
    rospy.Subscriber('odometry/filtered', Odometry, odometryCb)
    timer = rospy.Timer(rospy.Duration(0.02), timer_callback)

    rospy.spin()
    timer.shutdown()
