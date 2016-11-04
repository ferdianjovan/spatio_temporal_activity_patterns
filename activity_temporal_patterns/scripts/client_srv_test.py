#!/usr/bin/env python

import rospy
import argparse
from activity_temporal_patterns.srv import ActivityEstimateSrv
from activity_temporal_patterns.srv import ActivityBestTimeEstimateSrv


class ActivityClientSrv(object):

    def __init__(self, start_time, duration):
        self.service_estimate = rospy.ServiceProxy(
            "/activity_counter/activity_estimate",
            ActivityEstimateSrv
        )
        self.service_estimate.wait_for_service()
        self.service_best_time_estimate = rospy.ServiceProxy(
            "/activity_counter/activity_best_time_estimate",
            ActivityBestTimeEstimateSrv
        )
        start = rospy.Time.now() - rospy.Duration(start_time)
        end = start + rospy.Duration(duration)
        result = self.service_estimate(start, end, True, False)
        print "Activity Estimate With Upper Bound: ", result
        result = self.service_estimate(start, end, False, False)
        print "Activity Estimate Without Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, True)
        print "Activity Best Time Estimate With Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, False)
        print "Activity Best Time Estimate Without Upper Bound: ", result


if __name__ == "__main__":
    rospy.init_node("test_peetrap_srv")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("start_time", help="n seconds back for starting time")
    parser_arg.add_argument("duration", help="duration (in seconds)")
    args = parser_arg.parse_args()
    ActivityClientSrv(int(args.start_time), int(args.duration))
