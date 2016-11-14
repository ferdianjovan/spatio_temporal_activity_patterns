#!/usr/bin/env python

import copy
import rospy
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from activity_temporal_patterns.detect_peaks import detect_peaks
from activity_temporal_patterns.online_counter import ActivityCounter
from activity_temporal_patterns.srv import ActivityEstimateSrv, ActivityEstimateSrvResponse
from activity_temporal_patterns.srv import ActivityBestTimeEstimateSrv, ActivityBestTimeEstimateSrvResponse


class ActivityCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating activity counter...")
        self.time_window = rospy.Duration(
            rospy.get_param("~time_window", 10)*60
        )
        self.time_increment = rospy.Duration(
            rospy.get_param("~time_increment", 1)*60
        )
        self.counter = ActivityCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
            rospy.get_param("~update_every", 60)
        )
        self.counter.load_from_db()
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/activity_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/activity_estimate' % rospy.get_name(),
            ActivityEstimateSrv, self._srv_cb
        )
        rospy.loginfo("Preparing %s/activity_best_time_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/activity_best_time_estimate' % rospy.get_name(),
            ActivityBestTimeEstimateSrv, self._srv_best_cb
        )
        rospy.Service(
            '%s/restart' % rospy.get_name(), Empty, self._restart_srv_cb
        )
        rospy.sleep(0.1)

    def _restart_srv_cb(self, msg):
        self.counter.request_stop_update()
        self.counter = ActivityCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
            rospy.get_param("~update_every", 60)
        )
        # load_from_db for the current version
        self.counter.load_from_db()
        self.counter.continuous_update()
        return EmptyResponse()

    def _srv_cb(self, msg):
        rois = list()
        counts = list()
        activities = list()
        result = dict()

        rospy.loginfo(
            "Got a request to estimate ongoing activities within %d and %d..."
            % (msg.start_time.secs, msg.end_time.secs)
        )
        if (msg.end_time - msg.start_time).secs % self.time_window.secs != 0:
            adder = (msg.end_time - msg.start_time).secs / self.time_window.secs
            adder = rospy.Duration((adder+1) * self.time_window.secs)
            msg.end_time = msg.start_time + adder

        start = msg.start_time
        while start < msg.end_time:
            rois_acts = self.counter.retrieve_from_to(
                # scale might not be needed
                start, start + self.time_window, msg.upper_bound, msg.scale
            )
            for roi, acts in rois_acts.iteritems():
                for act, numbers in acts.iteritems():
                    if len(numbers) == 0:
                        continue
                    assert len(numbers) == 1, "len:%d (len should be 1), start:%d, end:%d" % (
                        len(numbers), start.secs, (start + self.time_window).secs
                    )
                    if str(roi)+"-"+str(act) not in result:
                        result[str(roi)+"-"+str(act)] = 0
                    result[str(roi)+"-"+str(act)] += sum(numbers.values())
            start = start + self.time_window

        for key, val in result.iteritems():
            roi, act = key.split("-")
            rois.append(roi)
            counts.append(val)
            activities.append(act)
        estimate = ActivityEstimateSrvResponse(rois, activities, counts)
        rospy.loginfo("Activity estimate: %s" % estimate)
        return estimate

    def _srv_best_cb(self, msg):
        rospy.loginfo(
            "Got a request to find %s highest intensity of activities within %d and %d..."
            % (str(msg.number_of_estimates), msg.start_time.secs, msg.end_time.secs)
        )
        times, rois, estimates = self._find_peak_estimates(msg)
        if len(times) == 0 and len(rois) == 0 and len(estimates) == 0:
            times, rois, estimates = self._find_highest_estimates(msg)
        estimate = ActivityBestTimeEstimateSrvResponse(times, rois, estimates)
        rospy.loginfo("Activity estimate: %s" % str(estimate))
        return estimate

    def _find_peak_estimates(self, msg):
        estimates = list()
        rois_activity = self.counter.retrieve_from_to(
            msg.start_time, msg.end_time, msg.upper_bound
        )
        for roi, acts in rois_activity.iteritems():
            roi_level_rates = list()
            roi_level_times = list()
            for act, val in acts.iteritems():
                times = sorted(val.keys())
                if roi_level_times != list():
                    assert len(times) == len(roi_level_times), "Length of data between two activities is not same"
                roi_level_times = copy.deepcopy(times)
                rates = [val[key] for key in times]
                if roi_level_rates == list() and len(rates) > 0:
                    # assuming all rates have the same length!!!
                    roi_level_rates = np.zeros(len(rates))
                roi_level_rates += np.array(rates)
            if roi_level_rates != list():
                roi_level_rates = roi_level_rates.to_list()
            peak_idx = detect_peaks(roi_level_rates)
            tmp = [(
                rospy.Time(int(roi_level_times[idx].split("-")[0])),
                roi, roi_level_rates[idx]
            ) for idx in peak_idx]
            estimates.extend(tmp)
        estimates = sorted(estimates, key=lambda i: i[3], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        if len(estimates) > 0:
            return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2], zip(*estimates)[3]
        else:
            return list(), list(), list(), list()

    def _find_highest_estimates(self, msg):
        estimates = list()  # each point is a tuple of (time, region, estimate)
        start = msg.start_time
        while start + self.time_window <= msg.end_time:
            rois_activity = self.counter.retrieve_from_to(
                start, start + self.time_window, msg.upper_bound
            )
            rois_activity = {
                roi: sum(
                    [sum(val.values()) for val in acts.itervalues() if len(val) > 0]
                ) for roi, acts in rois_activity.iteritems()
            }
            # if len(rois_activity.values()) > 0 and roi_activitiy.values()[0]
            for i in range(min(len(rois_activity), 3)):
                estimate = max(rois_activity.values())
                roi = rois_activity.keys()[rois_activity.values().index(estimate)]
                estimates.append((start, roi, estimate))
                del rois_activity[roi]
            start = start + self.time_increment
        estimates = sorted(estimates, key=lambda i: i[2], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        if len(estimates) > 0:
            return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2]
        else:
            return list(), list(), list()


if __name__ == '__main__':
    rospy.init_node("activity_counter")
    ActivityCounterService()
    rospy.spin()
