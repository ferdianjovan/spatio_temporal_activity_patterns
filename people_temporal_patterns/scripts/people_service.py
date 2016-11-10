#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from people_temporal_patterns.online_counter import PeopleCounter
from people_temporal_patterns.srv import PeopleEstimateSrv, PeopleEstimateSrvResponse
from people_temporal_patterns.srv import PeopleBestTimeEstimateSrv, PeopleBestTimeEstimateSrvResponse


class PeopleCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating people counter...")
        self.time_window = rospy.Duration(
            rospy.get_param("~time_window", 10)*60
        )
        self.time_increment = rospy.Duration(
            rospy.get_param("~time_increment", 1)*60
        )
        self.counter = PeopleCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080),
        )
        self.counter.load_from_db()
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/people_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/people_estimate' % rospy.get_name(), PeopleEstimateSrv, self._srv_cb
        )
        rospy.loginfo("Preparing %s/people_best_time_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/people_best_time_estimate' % rospy.get_name(),
            PeopleBestTimeEstimateSrv, self._srv_best_cb
        )
        rospy.Service(
            '%s/restart' % rospy.get_name(), Empty, self._restart_srv_cb
        )
        rospy.sleep(0.1)

    def _restart_srv_cb(self, msg):
        self.counter.request_stop_update()
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
                start, start + self.time_window, msg.upper_bound, msg.scale
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

    def _srv_best_cb(self, msg):
        rospy.loginfo(
            "Got a request to find %s highest number of people within %d and %d..."
            % (str(msg.number_of_estimates), msg.start_time.secs, msg.end_time.secs)
        )
        times, rois, estimates = self._find_highest_estimates(msg)
        estimate = PeopleBestTimeEstimateSrvResponse(times, rois, estimates)
        rospy.loginfo("People estimate: %s" % str(estimate))
        return estimate

    def _find_highest_estimates(self, msg):
        estimates = list()  # each point is a tuple of (time, region, estimate)
        start = msg.start_time
        while start + self.time_window <= msg.end_time:
            rois_people = self.counter.retrieve_from_to(
                start, start + self.time_window, msg.upper_bound
            )
            rois_people = {
                roi: sum(val.values()) for roi, val in rois_people.iteritems() if len(val) > 0
            }
            if len(rois_people.values()) > 0:
                # for each time point, pick the highest 3 regions
                for i in range(min(len(rois_people.values()), 3)):
                    estimate = max(rois_people.values())
                    roi = rois_people.keys()[rois_people.values().index(estimate)]
                    estimates.append((start, roi, estimate))
                    del rois_people[roi]
            start = start + self.time_increment
        estimates = sorted(estimates, key=lambda i: i[2], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        if len(estimates) > 0:
            return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2]
        else:
            return list(), list(), list()

    def continuous_update(self):
        rospy.loginfo("Continuously counting people...")
        self.counter.continuous_update()


if __name__ == '__main__':
    rospy.init_node("people_counter")
    pcs = PeopleCounterService()
    pcs.continuous_update()
    rospy.spin()
