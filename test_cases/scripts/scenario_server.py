#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 11:10:28 2015

@author: cdondrup
"""

import sys
import socket
import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
import yaml
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
import actionlib
import actionlib_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from test_cases.srv import Load, LoadResponse, Run, RunResponse
from roslib.packages import find_resource
import time

HOST = '127.0.0.1'
PORT = 4000


class ScenarioServer(object):
    _id = 0
    _human_success = True
    _min_distance_to_human = 1000.0
    _loaded = False
    _robot_poses = []

    def __init__(self, name):
        rospy.loginfo("Starting static wall scenario")
        rospy.Service("~load", Load, self.load_scenario)
        rospy.Service("~reset", Empty, self.reset)
        rospy.Service("~start", Run, self.start)

        rospy.loginfo("Starting move_base client...")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... done")
        rospy.loginfo("All done")

    def human_callback(self, msg):
        self._human_success = msg.data > self._min_distance if self._human_success else False
        self._min_distance_to_human = msg.data if msg.data < self._min_distance_to_human else self._min_distance_to_human

    def robot_callback(self, msg):
        self._robot_poses.append(msg)

    def _connect_port(self, port):
        """ Establish the connection with the given MORSE port"""
        sock = None

        for res in socket.getaddrinfo(HOST, port, socket.AF_UNSPEC, socket.SOCK_STREAM):
            af, socktype, proto, canonname, sa = res
            try:
                sock = socket.socket(af, socktype, proto)
            except socket.error:
                sock = None
                continue
            try:
                sock.connect(sa)
            except socket.error:
                sock.close()
                sock = None
                continue
            break

        return sock

    def _translate(self, x, y):
        return x+2.5, y+2.8

    def _clear_costmaps(self):
        try:
            s = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            s.wait_for_service()
            s()
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def _init_nav(self, pose, rot):
        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(1.0)
        initialPose = PoseWithCovarianceStamped()
        initialPose.header.stamp = rospy.Time.now()
        initialPose.header.frame_id = 'map'
        initialPose.pose.pose.position.x = pose["x"]
        initialPose.pose.pose.position.y = pose["y"]
        initialPose.pose.pose.position.z = pose["z"]
        initialPose.pose.pose.orientation.z = rot["z"]
        initialPose.pose.pose.orientation.w = rot["w"]
        p_cov = np.array([0.0]*36)
        initialPose.pose.covariance = tuple(p_cov.ravel().tolist())
        pub.publish(initialPose)

    def _get_set_object_pose_command(self, agent, id, x, y, rot):
            return 'id%d simulation set_object_pose ["%s", "[%f, %f, 0.1]", "[%f, %f, %f, %f]"]\n' \
            % (id, agent, x, y, \
            rot["w"], rot["x"], rot["y"], rot["z"])

    def reset(self, req):
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return EmptyResponse()

        self.client.cancel_all_goals()
        sock = self._connect_port(PORT)
        if not sock:
                sys.exit(1)

        rospy.loginfo("socket connected")

        sock.send(
            self._get_set_object_pose_command(
                "robot",
                self._id,
                self._robot_tx,
                self._robot_ty,
                self._robot_start_rot
            )
        )
        self._id += 1

        sock.send(
            self._get_set_object_pose_command(
                "human",
                self._id,
                self._human_tx,
                self._human_ty,
                self._human_start_rot
            )
        )
        self._id += 1

        sock.close()
        self._init_nav(self._robot_start_pose, self._robot_start_rot)
        self._clear_costmaps()
        return EmptyResponse()

    def start(self, req):
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return RunResponse(False, False)

        self._human_success = True
        rospy.Subscriber("/human_robot_distance", Float64, self.human_callback)
        self._robot_poses = []
        rospy.Subscriber("/robot_pose", Pose, self.robot_callback)
        t = time.time()
        self.client.send_goal_and_wait(self._goal, execute_timeout=rospy.Duration(self._timeout))
        elapsed = time.time() - t
        res = self.client.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED
        self.client.cancel_all_goals()
        distance = self.get_distance_travelled(self._robot_poses)
        return RunResponse(
            nav_success=res,
            human_success=self._human_success,
            min_distance_to_human=self._min_distance_to_human,
            distance_travelled=distance,
            travel_time=elapsed,
            mean_speed=distance/elapsed
        )

    def get_distance_travelled(self, poses):
        distance = 0.0
        for idx in range(len(poses))[1:]:
            distance += np.sqrt(
                [((poses[idx].position.x - poses[idx-1].position.x)**2) \
                + ((poses[idx].position.y - poses[idx-1].position.y)**2)]
            )
        return distance

    def load_scenario(self, req):
        conf_file = find_resource("test_cases", req.scenario+".yaml")[0]
        rospy.loginfo("Reading config file: %s ..." % conf_file)
        with open(conf_file,'r') as f:
            conf = yaml.load(f)
        rospy.loginfo(" ... done")

        self._robot_start_pose = conf["robot_start_pose"]["position"]
        self._robot_tx, self._robot_ty = self._translate(
            self._robot_start_pose["x"],
            self._robot_start_pose["y"]
        )
        self._robot_start_rot = conf["robot_start_pose"]["orientation"]

        self._goal = MoveBaseGoal()
        self._goal.target_pose.header.frame_id = "map"
        self._goal.target_pose.pose.position.x = conf["robot_end_pose"]["position"]["x"]
        self._goal.target_pose.pose.position.y = conf["robot_end_pose"]["position"]["y"]
        self._goal.target_pose.pose.orientation.w = conf["robot_end_pose"]["orientation"]["w"]
        self._goal.target_pose.pose.orientation.z = conf["robot_end_pose"]["orientation"]["z"]

        self._human_start_pose = conf["human_start_pose"]["position"]
        self._human_tx, self._human_ty = self._translate(
            self._human_start_pose["x"],
            self._human_start_pose["y"]
        )
        self._human_start_rot = conf["human_start_pose"]["orientation"]

        self._timeout = conf["success_metrics"]["nav_timeout"]
        self._min_distance = conf["success_metrics"]["human_distance"]
        self._loaded = True
        self.reset(None)
        return LoadResponse()

if __name__ == "__main__":
    rospy.init_node("scenario_server")
    s = ScenarioServer(rospy.get_name())
    rospy.spin()
