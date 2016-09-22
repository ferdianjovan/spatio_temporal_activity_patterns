#!/usr/bin/env python

import rospy
import argparse
from strands_exploration_msgs.srv import GetExplorationTasks


class ActivityClientSrv(object):

    def __init__(self, time_window, back_minutes):
        self.service = rospy.ServiceProxy(
            "/arms/activity_rcmd_srv",
            GetExplorationTasks
        )
        self.service.wait_for_service()
        start = rospy.Time.now() - rospy.Duration(time_window*60+60)
        end = start + rospy.Duration(60*back_minutes)
        result = self.service(start, end)
        print result


if __name__ == "__main__":
    rospy.init_node("test_activity_recommender")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("time_window", help="n minutes time window")
    parser_arg.add_argument("minutes", help="duration")
    args = parser_arg.parse_args()
    ActivityClientSrv(int(args.time_window), int(args.minutes))
