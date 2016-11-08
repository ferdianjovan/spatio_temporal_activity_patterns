#!/usr/bin/env python

import rospy
from simple_change_detector.msg import ChangeDetectionMsg
from spectral_processes.wrapper import SpectralPoissonWrapper
from simple_change_detector.stationary_shifting_detection import StationaryShiftingDetection


class DetectorManager(object):

    def __init__(self):
        img_topic = rospy.get_param("~img_topic", "/head_xtion/rgb/image_raw")
        sample_size = rospy.get_param("~sample_size", 20)
        wait_time = rospy.get_param("~wait_time", 5)
        self.ssd = StationaryShiftingDetection(
            topic_img=img_topic, sample_size=sample_size, wait_time=wait_time
        )
        if rospy.get_param("~with_temporal_model", False):
            time_window = rospy.get_param("~time_window", 10)
            time_increment = rospy.get_param("~time_increment", 1)
            periodic_cycle = rospy.get_param("~periodic_cycle", 10080)
            self.process = SpectralPoissonWrapper(
                rospy.get_name()+"/detections", ChangeDetectionMsg, "object_area",
                None, not_value=True, meta_info=dict(), max_count=1, window=time_window,
                increment=time_increment, periodic_cycle=periodic_cycle
            )
        self.ssd.publish_shifting_message()
        rospy.spin()


if __name__ == '__main__':
    img_topic = rospy.get_param("~img_topic", "/head_xtion/rgb/image_raw")
    tmp = img_topic.split("/")
    rospy.init_node("%s_change_detection" % tmp[1])
    dm = DetectorManager()
