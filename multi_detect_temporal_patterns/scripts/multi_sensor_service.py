#!/usr/bin/env python

import rospy
import numpy as np
from multi_detect_temporal_patterns.detect_peaks import detect_peaks
from multi_detect_temporal_patterns.online_counter import MultiDetectionCounter
from multi_detect_temporal_patterns.srv import BestTimeEstimateSrv, BestTimeEstimateSrvResponse


class MultiSensorCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating multi sensor counter...")
        self.time_window = rospy.Duration(
            rospy.get_param("~time_window", 10)*60
        )
        self.time_increment = rospy.Duration(
            rospy.get_param("~time_increment", 1)*60
        )
        self.counter = MultiDetectionCounter(
            rospy.get_param("~soma_config", "poisson_activity"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 1440),
        )
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/best_time_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/best_time_estimate' % rospy.get_name(),
            BestTimeEstimateSrv, self._srv_best_cb
        )
        rospy.sleep(0.1)

    def _srv_best_cb(self, msg):
        rospy.loginfo(
            "Got a request to find %s highest intensity of detections within %d and %d..."
            % (str(msg.number_of_estimates), msg.start_time.secs, msg.end_time.secs)
        )
        times, rois, estimates = self._find_peak_estimates(msg)
        if len(times) == 0 and len(rois) == 0 and len(estimates) == 0:
            times, rois, estimates = self._find_highest_estimates(msg)
        estimate = BestTimeEstimateSrvResponse(times, rois, estimates)
        rospy.loginfo("Multi-sensor estimate: %s" % str(estimate))
        return estimate

    def _find_peak_estimates(self, msg):
        estimates = list()
        estimate_per_roi = self.counter.retrieve_from_to(
            msg.start_time, msg.end_time, msg.upper_bound
        )
        for roi, val in estimate_per_roi.iteritems():
            if len(val) == 0:
                continue
            # dict.keys() and dict.values() should give the same order of items
            times = sorted(val.keys())
            rates = [val[key] for key in times]
            peak_idx = detect_peaks(rates, mpd=len(times)/10)
            tmp = [(
                rospy.Time(int(times[idx].split("-")[0])), roi, rates[idx]
            ) for idx in peak_idx]
            estimates.extend(tmp)
        estimates = sorted(estimates, key=lambda i: i[2], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        if len(estimates) > 0:
            return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2]
        else:
            return list(), list(), list()

    def _find_highest_estimates(self, msg):
        estimates = list()  # each point is a tuple of (time, region, estimate)
        start = msg.start_time
        while start + self.time_window <= msg.end_time:
            estimate_per_roi = self.counter.retrieve_from_to(
                start, start + self.time_window, msg.upper_bound
            )
            estimate_per_roi = {
                roi: sum(
                    val.values()
                ) for roi, val in estimate_per_roi.iteritems() if len(val) > 0
            }
            # for each time point, pick the highest 3 regions
            for i in range(min(len(estimate_per_roi), 3)):
                estimate = max(estimate_per_roi.values())
                roi = estimate_per_roi.keys()[estimate_per_roi.values().index(estimate)]
                estimates.append((start, roi, estimate))
                del estimate_per_roi[roi]
            start = start + self.time_increment
        estimates = sorted(estimates, key=lambda i: i[2], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        if len(estimates) > 0:
            return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2]
        else:
            return list(), list(), list()


if __name__ == '__main__':
    rospy.init_node("multi_sensor_counting_process")
    MultiSensorCounterService()
    rospy.spin()
