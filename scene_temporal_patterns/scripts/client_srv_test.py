#!/usr/bin/env python

import rospy
import argparse
from scene_temporal_patterns.srv import SceneBestTimeEstimateSrv, SceneEstimateSrv


class SceneClientSrv(object):

    def __init__(self, start_time, duration):
        self.service_estimate = rospy.ServiceProxy(
            "/scene_counter/scene_estimate", SceneEstimateSrv
        )
        self.service_estimate.wait_for_service()
        self.service_best_time_estimate = rospy.ServiceProxy(
            "/scene_counter/scene_best_time_estimate", SceneBestTimeEstimateSrv
        )
        self.service_best_time_estimate.wait_for_service()
        start = rospy.Time.now() - rospy.Duration(start_time)
        end = start + rospy.Duration(duration)
        result = self.service_estimate(start, end, True, False)
        print "Scene Estimate With Upper Bound: ", result
        result = self.service_estimate(start, end, False, False)
        print "Scene Estimate Without Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, True)
        print "Scene Estimate With Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, False)
        print "Scene Estimate Without Upper Bound: ", result


if __name__ == "__main__":
    rospy.init_node("test_scitrap_srv")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("start_time", help="n seconds back for starting time")
    parser_arg.add_argument("duration", help="duration (in seconds)")
    args = parser_arg.parse_args()
    SceneClientSrv(int(args.start_time), int(args.duration))
