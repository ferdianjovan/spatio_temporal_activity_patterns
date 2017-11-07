#!/usr/bin/env python

import rospy
from detection_observation.ubd_count import UBDCountObservation
from detection_observation.scene_count import SceneCountObservation
from detection_observation.trajectory_count import TrajectoryCountObservation


class DetectionCounter(object):

    def __init__(
        self, config, increment=rospy.Duration(60),
        max_count_per_increment=1
    ):
        self.time_increment = increment
        self.scene_counter = SceneCountObservation(
            config, increment=increment,
            max_count_per_increment=max_count_per_increment
        )
        self.trajectory_counter = TrajectoryCountObservation(
            config, increment=increment,
            max_count_per_increment=max_count_per_increment
        )
        self.ubd_counter = UBDCountObservation(
            config, increment=increment,
            max_count_per_increment=max_count_per_increment
        )
        rospy.loginfo("Start counting and storing sensor detections...")
        rospy.Timer(self.count_and_store, self.time_increment)

    def count_and_store(self):
        end_time = rospy.Time(
            ((rospy.Time.now().secs / 60) * 60)
        )
        start_time = end_time - self.time_increment
        self.scene_counter.store_scene_observation_per_time_increment(
            start_time, end_time
        )
        self.trajectory_counter.store_trajectory_observation_per_time_increment(
            start_time, end_time
        )
        self.ubd_counter.store_ubd_observation_per_time_increment(
            start_time, end_time
        )


if __name__ == '__main__':
    rospy.init_node("sensor_detection_counter")
    sensor_counter = DetectionCounter(
        rospy.get_param("~soma_config", "poisson_activity"),
        increment=rospy.Duration(rospy.get_param("~time_increment", 1)*60),
        max_count_per_increment=rospy.get_param("~max_count_per_increment", 1)
    )
    rospy.spin()
