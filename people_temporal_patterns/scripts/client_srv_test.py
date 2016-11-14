#!/usr/bin/env python

import rospy
import argparse
from people_temporal_patterns.srv import PeopleEstimateSrv, PeopleBestTimeEstimateSrv


class PeopleClientSrv(object):

    def __init__(self, start_time, duration):
        self.service_estimate = rospy.ServiceProxy(
            "/people_counter/people_estimate", PeopleEstimateSrv
        )
        self.service_estimate.wait_for_service()
        self.service_best_time_estimate = rospy.ServiceProxy(
            "/people_counter/people_best_time_estimate", PeopleBestTimeEstimateSrv
        )
        self.service_best_time_estimate.wait_for_service()
        start = rospy.Time.now() - rospy.Duration(start_time)
        end = start + rospy.Duration(duration)
        result = self.service_estimate(start, end, True, False)
        print "People Estimate With Upper Bound: ", result
        result = self.service_estimate(start, end, False, False)
        print "People Estimate Without Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, True)
        print "People Estimate With Upper Bound: ", result
        result = self.service_best_time_estimate(start, end, 20, False)
        print "People Estimate Without Upper Bound: ", result


if __name__ == "__main__":
    rospy.init_node("test_peetrap_srv")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("start_time", help="n seconds back for starting time")
    parser_arg.add_argument("duration", help="duration (in seconds)")
    args = parser_arg.parse_args()
    PeopleClientSrv(int(args.start_time), int(args.duration))
