#!/usr/bin/env python

import time
import rospy
import datetime
from region_observation.online_observe import OnlineRegionObservation
from region_observation.offline_observe import OfflineRegionObservation


class Observation(object):

    def __init__(self, is_online=True):
        if is_online:
            self.online(
                rospy.get_param("~soma_config", "activity_exploration"),
                rospy.get_param("~time_increment", 1)
            )
        else:
            self.offline(
                rospy.get_param("~soma_config", "activity_exploration"),
                rospy.get_param("~time_increment", 1)
            )

    def online(self, config, increment):
        self.ro = OnlineRegionObservation(rospy.get_name(), config, increment)
        self.ro.observe()
        rospy.spin()

    def offline(self, config, increment):
        self.ro = OfflineRegionObservation(config, increment)
        start_time = raw_input("Start date 'year month day hour minute':")
        start_time = start_time.split(" ")
        start_time = datetime.datetime(
            int(start_time[0]), int(start_time[1]), int(start_time[2]),
            int(start_time[3]), int(start_time[4])
        )
        start_time = rospy.Time(time.mktime(start_time.timetuple()))
        end_time = raw_input("End date 'year month day hour minute':")
        end_time = end_time.split(" ")
        end_time = datetime.datetime(
            int(end_time[0]), int(end_time[1]), int(end_time[2]),
            int(end_time[3]), int(end_time[4])
        )
        end_time = rospy.Time(time.mktime(end_time.timetuple()))
        self.ro.calculate_observation(start_time, end_time)


if __name__ == '__main__':
    rospy.init_node("region_observation")
    obs = Observation(rospy.get_param("~online", True))
