#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from threading import Thread
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException

class DistanceCalc(object):
    def __init__(self, name):
        rospy.loginfo("Starting node %s..." % name)
        self.tf = TransformListener()
        self.human_pose = None
        self.robot_pose = None
        self.target_tf = rospy.get_param("~target_frame", "/map")
        self.pub = rospy.Publisher("/human_robot_distance", Float64, queue_size=1)
        rospy.Subscriber("/human/transformed", PoseStamped, self.human_callback)
        rospy.Subscriber("/robot_pose", Pose, self.robot_callback)
        _distance_calc_thread = Thread(target=self.distance_calc, args=())
        _distance_calc_thread.start()
        rospy.loginfo(" ... done")

    def human_callback(self, msg):
        try:
            t = self.tf.getLatestCommonTime(self.target_tf, msg.header.frame_id)
            msg.header.stamp = t
            self.human_pose = self.tf.transformPose(self.target_tf, msg)
        except (Exception, LookupException, ConnectivityException, ExtrapolationException) as e:
            rospy.logwarn(e)

    def robot_callback(self, msg):
        self.robot_pose = msg

    def distance_calc(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.human_pose and self.robot_pose:
                self.pub.publish(
                    np.sqrt(
                        [((self.human_pose.pose.position.x - self.robot_pose.position.x)**2) \
                        + ((self.human_pose.pose.position.y - self.robot_pose.position.y)**2)]
                    )
                )
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("distance_calc")
    d = DistanceCalc(rospy.get_name())
    rospy.spin()