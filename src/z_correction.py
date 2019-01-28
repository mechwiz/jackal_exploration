#!/usr/bin/env python
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class z_correction:
    def __init__(self):
        self.odom_corrected = Odometry()
        self.transtamp = TransformStamped()
        self.tf_new = TransformStamped()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.odm_filter = rospy.Subscriber('odometry/filtered', Odometry, self.odometryCb)
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        self.pub = rospy.Publisher('/odometry/corrected', Odometry, queue_size=30)


    def odometryCb(self,data):
        self.odom_corrected = data
        self.odom_corrected.header.frame_id = "odom_corrected"
        self.odom_corrected.child_frame_id = "base_link"
        self.odom_corrected.pose.pose.position.z = 0
        self.pub.publish(self.odom_corrected)

    def timer_callback(self,event):
        try:
            self.transtamp = self.tf_buffer.lookup_transform('odom','base_link',rospy.Time())
        except tf2_ros.LookupException:
            rospy.loginfo("Missed")
            pass

        z_offset = self.transtamp.transform.translation.z

        self.tf_new.header.stamp = rospy.Time.now()
        self.tf_new.header.frame_id = "odom_corrected"
        self.tf_new.child_frame_id = "odom"
        self.tf_new.transform.translation.x = 0
        self.tf_new.transform.translation.y = 0
        self.tf_new.transform.translation.z = -z_offset
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.tf_new.transform.rotation.x = q[0]
        self.tf_new.transform.rotation.y = q[1]
        self.tf_new.transform.rotation.z = q[2]
        self.tf_new.transform.rotation.w = q[3]

        self.br.sendTransform(self.tf_new)

if __name__ == "__main__":
    rospy.init_node('z_correction') #make node
    zc = z_correction()

    rospy.spin()
