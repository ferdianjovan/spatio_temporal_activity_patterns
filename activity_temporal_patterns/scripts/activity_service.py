#!/usr/bin/env python

import rospy
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
        self.service = rospy.Service(
            '%s/activity_estimate' % rospy.get_name(),
            ActivityEstimateSrv, self._srv_cb
        )
        rospy.loginfo("Preparing %s/activity_best_time_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/activity_best_time_estimate' % rospy.get_name(),
            ActivityBestTimeEstimateSrv, self._srv_best_cb
        )
        rospy.sleep(0.1)

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
        times, rois, estimates = self._find_highest_estimates(msg)
        estimate = ActivityBestTimeEstimateSrvResponse(times, rois, estimates)
        rospy.loginfo("Activity estimate: %s" % str(estimate))
        return estimate

    def _find_highest_estimates(self, msg):
        estimates = list()  # each point is a tuple of (time, region, estimate)
        start = msg.start_time
        while start + self.time_window <= msg.end_time:
            rois_activity = self.counter.retrieve_from_to(
                start, start + self.time_window, msg.upper_bound
            )
            rois_activity = {
                roi: sum(
                    [sum(val.values()) for val in acts.itervalues()]
                ) for roi, acts in rois_activity.iteritems()
            }
            for i in range(3):  # for each time point, pick the highest 3 regions
                estimate = max(rois_activity.values())
                roi = rois_activity.keys()[rois_activity.values().index(estimate)]
                estimates.append((start, roi, estimate))
                del rois_activity[roi]
            start = start + self.time_increment
        estimates = sorted(estimates, key=lambda i: i[2], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2]


if __name__ == '__main__':
    rospy.init_node("activity_counter")
    acs = ActivityCounterService()
    acs.continuous_update()
    rospy.spin()
