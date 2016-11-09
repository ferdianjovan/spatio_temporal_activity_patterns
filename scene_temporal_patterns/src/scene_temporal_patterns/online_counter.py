#!/usr/bin/env python

import copy
import time
import math
import rospy
import argparse
import datetime
import threading
from simple_change_detector.msg import ChangeDetectionMsg
from spectral_processes.processes import SpectralPoissonProcesses
from region_observation.observation_proxy import RegionObservationProxy
from region_observation.util import create_line_string, is_intersected, get_soma_info


class SceneCounter(object):

    def __init__(
        self, config, window=10, increment=1, periodic_cycle=10080
    ):
        temp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        self._start_time = rospy.Time(time.mktime(temp.timetuple()))
        self._is_stop_requested = False
        self._is_stopped = False
        self._lock = threading.Lock()
        # trajectories subscriber
        self.change_detections = list()
        self._scene_subs = rospy.Subscriber(
            rospy.get_param("~scene_topic", "/change_detection/detections"),
            ChangeDetectionMsg, self._pt_cb, None, 10
        )
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, self.config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        self.periodic_cycle = periodic_cycle
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = {
            roi: SpectralPoissonProcesses(window, increment, periodic_cycle) for roi in self.regions.keys()
        }

    def request_stop_update(self):
        self._is_stop_requested = True
        self._scene_subs.unregister()
        self._wait_to_stop()

    def _wait_to_stop(self):
        while not self._is_stopped:
            rospy.sleep(0.1)

    def request_continue_update(self):
        self._is_stop_requested = False
        self._scene_subs = rospy.Subscriber(
            rospy.get_param("~scene_topic", "/change_detection/detections"),
            ChangeDetectionMsg, self._pt_cb, None, 10
        )

    def retrieve_from_to(
        self, start_time, end_time, use_upper_confidence=False, scale=False
    ):
        result = dict()
        for roi, poisson in self.process.iteritems():
            result.update(
                # use upper confidence rate value
                {roi: poisson.retrieve(
                    start_time, end_time,
                    use_upper_confidence=use_upper_confidence, scale=scale
                )}
            )
        return result

    def load_from_db(self):
        rospy.loginfo("Retrieving people processes from database. It may take a while...")
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "scene"
            }
            self.process[roi].retrieve_from_mongo(meta)

    def store_to_db(self):
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "scene"
            }
            self.process[roi].store_to_mongo(meta)

    def _pt_cb(self, msg):
        if len(msg.object_centroids) == 0:
            return
        self._lock.acquire_lock()
        self.change_detections.append(msg)
        self._lock.release_lock()

    def continuous_update(self):
        self._is_stopped = False
        while not rospy.is_shutdown() and not self._is_stop_requested:
            now = rospy.Time.now()
            delta = (now - self._start_time)
            if delta > self.time_window:
                self.update()
            rospy.sleep(0.1)
        self._is_stopped = True

    def update(self):
        new_end = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        new_end = datetime.datetime(
            new_end.year, new_end.month, new_end.day, new_end.hour,
            new_end.minute
        )
        end_time = rospy.Time(time.mktime(new_end.timetuple()))
        region_observations = self.obs_proxy.load_msg(
            self._start_time, end_time, minute_increment=self.time_increment.secs/60
        )
        temp = copy.deepcopy(self.change_detections)
        rospy.loginfo("Total scene deviation counted so far is %d." % len(temp))
        rospy.loginfo("Maximum one moving objects per minute is allowed.")

        used_detections = list()
        count_per_region = dict()
        for observation in region_observations:
            count = 0
            for detection in temp:
                points = [
                    [point.x, point.y] for point in detection.object_centroids
                ]
                points = create_line_string(points)
                if is_intersected(self.regions[observation.region_id], points):
                    # it also must be within time boundaries
                    conditions = detection.header.stamp >= observation.start_from
                    # observation.until is secs.999999999999
                    conditions = conditions and detection.header.stamp <= observation.until
                    if conditions:
                        count += 1
                if (
                    detection.header.stamp < (self._start_time+self.time_increment)
                ) and (detection not in used_detections):
                    used_detections.append(detection)
            if count > 0 or observation.duration.secs >= 59:
                count = max(0, int(bool(count)))
                if observation.region_id not in count_per_region.keys():
                    count_per_region[observation.region_id] = 0
                count_per_region[observation.region_id] += count
        # update and save observation for that time
        for roi, count in count_per_region.iteritems():
            self.process[roi].update(self._start_time, count)
            self._store(roi, self._start_time)
        self._start_time = self._start_time + self.time_increment
        # remove detections that have been updated,
        # and update the current stored trajectories
        n = len(temp)
        temp = [i for i in temp if i not in used_detections]
        self._lock.acquire_lock()
        self.change_detections = temp + self.change_detections[n:]
        self._lock.release_lock()

    def _store(self, roi, start_time):
        self.process[roi]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "scene"
            }
        )


if __name__ == '__main__':
    rospy.init_node("online_scene_counter")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument('soma_config', help="Soma configuration")
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Fixed time window interval (in minute) for each Poisson distribution. Default is 10 minute."
    )
    parser.add_argument(
        "-m", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute."
    )
    parser.add_argument(
        "-p", dest="periodic_cycle", default="10080",
        help="Desired periodic cycle (in minute). Default is one week (10080 minutes)"
    )
    args = parser.parse_args()

    ppp = SceneCounter(
        args.soma_config, int(args.time_window), int(args.time_increment),
        int(args.periodic_cycle)
    )
    ppp.load_from_db()
    ppp.continuous_update()
    rospy.spin()
