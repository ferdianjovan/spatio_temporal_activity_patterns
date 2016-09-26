#!/usr/bin/env python

import rospy
from activity_temporal_patterns.online_counter import ActivityCounter
from activity_temporal_patterns.srv import ActivityEstimateSrv, ActivityEstimateSrvResponse


class ActivityCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating activity counter...")
        self.time_window = rospy.Duration(
            rospy.get_param("~time_window", 10)*60
        )
        self.counter = ActivityCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~activity_srv", "/activity_simulation/available_activities"),
            rospy.get_param("~activity_topic", "/activity_simulation/activities"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
        )
        self.counter.load_from_db()
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/activity_estimate..." % rospy.get_name())
        self.service = rospy.Service(
            '%s/activity_estimate' % rospy.get_name(),
            ActivityEstimateSrv, self._srv_cb
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
                start, start + self.time_window, msg.scale
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

    def spin(self):
        rospy.loginfo("Continuously observing activities...")
        self.counter.continuous_update()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("activity_counter")
    acs = ActivityCounterService()
    acs.spin()
