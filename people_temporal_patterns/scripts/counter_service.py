#!/usr/bin/env python

import rospy
import std_srvs.srv import Empty, EmptyResponse
from people_temporal_patterns.online_counter import PeopleCounter
from people_temporal_patterns.srv import PeopleEstimateSrv, PeopleEstimateSrvResponse


class PeopleCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating people counter...")
        self.time_window = rospy.Duration(
            rospy.get_param("~time_window", 10)*60
        )
        self.counter = PeopleCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
        )
        self.counter.load_from_db()
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/people_estimate..." % rospy.get_name())
        rospy.Service(
            '%s/people_estimate' % rospy.get_name(),
            PeopleEstimateSrv, self._srv_cb
        )
        rospy.Service(
            '%s/restart' % rospy.get_name(),
            Empty, self._restart_srv_cb
        )
        rospy.sleep(0.1)

    def _restart_srv_cb(self, msg):
        self.counter.request_stop_update()
        rospy.sleep(3)
        self.counter = PeopleCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
        )
        self.counter.continuous_update()
        return EmptyResponse()

    def _srv_cb(self, msg):
        rois = list()
        rates = list()
        result = dict()

        rospy.loginfo(
            "Got a request to estimate the number of people within %d and %d..."
            % (msg.start_time.secs, msg.end_time.secs)
        )
        if (msg.end_time - msg.start_time).secs % self.time_window.secs != 0:
            adder = (msg.end_time - msg.start_time).secs / self.time_window.secs
            adder = rospy.Duration((adder+1) * self.time_window.secs)
            msg.end_time = msg.start_time + adder

        start = msg.start_time
        while start < msg.end_time:
            # rois_rates = self.counter.retrieve_from_to(msg.start_time, msg.end_time, msg.scale)
            rois_people = self.counter.retrieve_from_to(
                # scale might not be needed
                start, start + self.time_window, msg.scale
            )
            for roi, estimates in rois_people.iteritems():
                if len(estimates) == 0:
                    continue
                assert len(estimates) == 1, "len:%d (len should be 1), start:%d, end:%d" % (
                    len(estimates), start.secs, (start+self.time_window).secs
                )
                if sum(estimates.values()) > 100:
                    print roi, estimates, start.secs, self.time_window.secs
                if roi not in result:
                    result[roi] = 0
                result[roi] += sum(estimates.values())
            start = start + self.time_window

        for roi, estimate in result.iteritems():
            rois.append(roi)
            rates.append(estimate)
        estimate = PeopleEstimateSrvResponse(rois, rates)
        rospy.loginfo("People estimate: %s" % str(estimate))
        return estimate

    def spin(self):
        rospy.loginfo("Continuously counting people...")
        self.counter.continuous_update()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("people_counter")
    pcs = PeopleCounterService()
    pcs.spin()
