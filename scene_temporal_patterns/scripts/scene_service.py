#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from scene_temporal_patterns.online_counter import SceneCounter
from scene_temporal_patterns.srv import SceneEstimateSrv, SceneEstimateSrvResponse
from scene_temporal_patterns.srv import SceneBestTimeEstimateSrv, SceneBestTimeEstimateSrvResponse


class SceneCounterService(object):

    def __init__(self):
        rospy.loginfo("Initiating scene counter...")
        self.time_window = rospy.Duration(
            rospy.get_param("~time_window", 10)*60
        )
        self.time_increment = rospy.Duration(
            rospy.get_param("~time_increment", 1)*60
        )
        self.counter = SceneCounter(
            rospy.get_param("~soma_config", "activity_exploration"),
            rospy.get_param("~time_window", 10),
            rospy.get_param("~time_increment", 1),
            rospy.get_param("~periodic_cycle", 10080)
        )
        self.counter.load_from_db()
        rospy.sleep(1)
        rospy.loginfo("Preparing %s/scene_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/scene_estimate' % rospy.get_name(), SceneEstimateSrv, self._srv_cb
        )
        rospy.loginfo("Preparing %s/scene_best_time_estimate service..." % rospy.get_name())
        rospy.Service(
            '%s/scene_best_time_estimate' % rospy.get_name(),
            SceneBestTimeEstimateSrv, self._srv_best_cb
        )
        rospy.Service(
            '%s/restart' % rospy.get_name(), Empty, self._restart_srv_cb
        )
        rospy.sleep(0.1)

    def _restart_srv_cb(self, msg):
        self.counter.request_stop_update()
        self.counter = SceneCounter(
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
            "Got a request to estimate the number of scene change within %d and %d..."
            % (msg.start_time.secs, msg.end_time.secs)
        )
        if (msg.end_time - msg.start_time).secs % self.time_window.secs != 0:
            adder = (msg.end_time - msg.start_time).secs / self.time_window.secs
            adder = rospy.Duration((adder+1) * self.time_window.secs)
            msg.end_time = msg.start_time + adder

        start = msg.start_time
        while start < msg.end_time:
            rois_scene = self.counter.retrieve_from_to(
                # scale might not be needed
                start, start + self.time_window, msg.upper_bound, msg.scale
            )
            for roi, estimates in rois_scene.iteritems():
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
        estimate = SceneEstimateSrvResponse(rois, rates)
        rospy.loginfo("Scene estimate: %s" % str(estimate))
        return estimate

    def _srv_best_cb(self, msg):
        rospy.loginfo(
            "Got a request to find %s highest number of scene changing within %d and %d..."
            % (str(msg.number_of_estimates), msg.start_time.secs, msg.end_time.secs)
        )
        times, rois, estimates = self._find_highest_estimates(msg)
        estimate = SceneBestTimeEstimateSrvResponse(times, rois, estimates)
        rospy.loginfo("Scene estimate: %s" % str(estimate))
        return estimate

    def _find_highest_estimates(self, msg):
        estimates = list()  # each point is a tuple of (time, region, estimate)
        start = msg.start_time
        while start + self.time_window <= msg.end_time:
            rois_scene = self.counter.retrieve_from_to(
                start, start + self.time_window, msg.upper_bound
            )
            rois_scene = {
                roi: sum(val.values()) for roi, val in rois_scene.iteritems() if len(val) > 0
            }
            if len(rois_scene.values()) > 0:
                # for each time point, pick the highest 3 regions
                for i in range(min(len(rois_scene.values()), 3)):
                    estimate = max(rois_scene.values())
                    roi = rois_scene.keys()[rois_scene.values().index(estimate)]
                    estimates.append((start, roi, estimate))
                    del rois_scene[roi]
            start = start + self.time_increment
        estimates = sorted(estimates, key=lambda i: i[2], reverse=True)
        estimates = estimates[:msg.number_of_estimates]
        if len(estimates) > 0:
            return zip(*estimates)[0], zip(*estimates)[1], zip(*estimates)[2]
        else:
            return list(), list(), list()

    def continuous_update(self):
        rospy.loginfo("Continuously observing scene changing...")
        self.counter.continuous_update()


if __name__ == '__main__':
    rospy.init_node("scene_counter")
    scs = SceneCounterService()
    scs.continuous_update()
    rospy.spin()
