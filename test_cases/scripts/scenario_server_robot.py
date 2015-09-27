#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 11:10:28 2015

@author: cdondrup
"""

import rospy
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
import yaml
import numpy as np
from geometry_msgs.msg import Pose
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
import actionlib
import actionlib_msgs
from test_cases.srv import Load, LoadResponse, Run, RunResponse
from roslib.packages import find_resource
import time
from bayes_people_tracker.msg import PeopleTracker


class ScenarioServer(object):
    _id = 0
    _human_success = True
    _loaded = False
    _move_human = True
    _robot_poses = []

    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        rospy.loginfo("Starting topological_navigation client...")
        self.client = actionlib.SimpleActionClient("topological_navigation", GotoNodeAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... done")

        rospy.Service("~load", Load, self.load_scenario)
        rospy.Service("~reset", Empty, self.reset)
        rospy.Service("~start", Run, self.start)
        rospy.loginfo("All done")

    def human_callback(self, msg):
        dist = msg.min_distance
        if dist == 0.0:
            return
        self._human_success = dist > self._min_distance if self._human_success else False
        self._min_distance_to_human = dist if dist < self._min_distance_to_human else self._min_distance_to_human

    def robot_callback(self, msg):
        self._robot_poses.append(msg)

    def _clear_costmaps(self):
        try:
            s = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            s.wait_for_service()
            s()
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def reset(self, req):
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return EmptyResponse()

        self.client.cancel_all_goals()

        g = GotoNodeGoal()
        g.target = self._waypoints["start"]
        g.no_orientation = False
        self.client.send_goal_and_wait(g)

        self._clear_costmaps()
        return EmptyResponse()

    def start(self, req):
        if not self._loaded:
            rospy.logfatal("No scenario loaded!")
            return RunResponse(False, False)

        self._human_success = True
        self._min_distance_to_human = 1000.0
        rospy.Subscriber("/people_tracker/positions", PeopleTracker, self.human_callback)
        self._robot_poses = []
        rospy.Subscriber("/robot_pose", Pose, self.robot_callback)
        g = GotoNodeGoal()
        g.target = self._waypoints["end"]
        g.no_orientation = True
        self.client.send_goal(g)
        t = time.time()
        self.client.wait_for_result(timeout=rospy.Duration(self._timeout))
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

        self._waypoints = conf["waypoints"]

        self._timeout = conf["success_metrics"]["nav_timeout"]
        self._min_distance = conf["success_metrics"]["human_distance"]
        self._loaded = True
        self.reset(None)

        return LoadResponse()

if __name__ == "__main__":
    rospy.init_node("scenario_server")
    s = ScenarioServer(rospy.get_name())
    rospy.spin()
