#!/usr/bin/env python

import rospy
import argparse
from strands_exploration_msgs.srv import GetExplorationTasks


class ActivityClientSrv(object):

    def __init__(self, start_time, duration):
        self.service = rospy.ServiceProxy(
            "/arms/activity_rcmd_srv",
            GetExplorationTasks
        )
        self.service.wait_for_service()
        start = rospy.Time.now() - rospy.Duration(start_time)
        end = start + rospy.Duration(duration)
        result = self.service(start, end)
        print result


if __name__ == "__main__":
    rospy.init_node("test_activity_recommender_srv")
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument("start_time", help="n seconds back for starting time")
    parser_arg.add_argument("duration", help="duration (in seconds)")
    args = parser_arg.parse_args()
    ActivityClientSrv(int(args.start_time), int(args.duration))
