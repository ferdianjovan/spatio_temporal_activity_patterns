#!/usr/bin/env python

import rospy
import argparse
from multi_detect_temporal_patterns.srv import BestTimeEstimateSrv


class MultiDetectClientSrv(object):

    def __init__(self, start_time, duration):
        self.service_best_time_estimate = rospy.ServiceProxy(
            "/multi_sensor_counting_process/best_time_estimate", BestTimeEstimateSrv
        )
        self.service_best_time_estimate.wait_for_service()
        start = rospy.Time.now() - rospy.Duration(start_time)
        end = start + rospy.Duration(duration)
        result = self.service_best_time_estimate(start, end, 20, True)
        print "Multi Detection Estimate With Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, False)
        print "Multi Detection Estimate Without Upper Bound: ", result


if __name__ == "__main__":
    rospy.init_node("popp_service_client")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("start_time", help="n seconds back for starting time")
    parser_arg.add_argument("duration", help="duration (in seconds)")
    args = parser_arg.parse_args()
    MultiDetectClientSrv(int(args.start_time), int(args.duration))
