#!/usr/bin/env python

import rospy
from people_temporal_patterns.online_counter import PeopleCounter
from people_temporal_patterns.srv import PeopleEstimateSrv, PeopleEstimateSrvResponse


class PeopleCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating people counter...")
        self.counter = PeopleCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
        )
        self.counter.load_from_db()
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/people_estimate..."  % rospy.get_name())
        self.service = rospy.Service(
            '%s/people_estimate' % rospy.get_name(),
            PeopleEstimateSrv, self._srv_cb
        )
        rospy.sleep(0.1)

    def _srv_cb(self, msg):
        rois = list()
        rates = list()

        rospy.loginfo(
            "Got a request to estimate the number of people within %d and %d..."
            % (msg.start_time.secs, msg.end_time.secs)
        )
        time_window = rospy.get_param("~time_window", 10)
        if (msg.end_time - msg.start_time).secs < time_window*60:
            rospy.logwarn(
                "Delta start and end time is small, widening the time window..."
            )
            msg.end_time = msg.start_time + rospy.Duration(time_window*60)

        rois_rates = self.counter.retrieve_from_to(msg.start_time, msg.end_time, msg.scale)
        for roi, rate in rois_rates.iteritems():  # need to check whether given 10 min delta, will it give 1 instance?
            divider = float(len(rate.values()))
            if divider == 0:
                divider = 1
            rois.append(roi)
            rates.append(sum(rate.values()) / divider)
        estimate = PeopleEstimateSrvResponse(rois, rates)
        return estimate

    def spin(self):
        rospy.loginfo("Continuously counting people...")
        self.counter.continuous_update()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node("people_counter")
    pcs = PeopleCounterService()
    pcs.spin()
