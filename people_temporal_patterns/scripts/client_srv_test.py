#!/usr/bin/env python

import rospy
import argparse
from people_temporal_patterns.srv import PeopleEstimateSrv


class PeopleClientSrv(object):

    def __init__(self, start_time, duration):
        self.service = rospy.ServiceProxy(
            "/people_counter/people_estimate",
            PeopleEstimateSrv
        )
        self.service.wait_for_service()
        start = rospy.Time.now() - rospy.Duration(start_time)
        end = start + rospy.Duration(duration)
        result = self.service(start, end, True, False)
        print "With Upper Bound: ", result
        result = self.service(start, end, False, False)
        print "Without Upper Bound: ", result


if __name__ == "__main__":
    rospy.init_node("test_peetrap_srv")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("start_time", help="n seconds back for starting time")
    parser_arg.add_argument("duration", help="duration (in seconds)")
    args = parser_arg.parse_args()
    PeopleClientSrv(int(args.start_time), int(args.duration))
