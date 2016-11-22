#!/usr/bin/env python

import time
import rospy
import argparse
import datetime
from region_observation.util import get_soma_info
from spectral_processes.processes import SpectralPoissonProcesses
from scene_temporal_patterns.scene_count import SceneRegionCount


class SceneCounter(object):

    def __init__(
        self, config, window=10, increment=1, periodic_cycle=10080
    ):
        rospy.loginfo("Starting scene processes...")
        self._start_time = None
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        # for each roi create PoissonProcesses
        rospy.loginfo(
            "Time window is %d minute with increment %d minute" % (
                window, increment
            )
        )
        self.periodic_cycle = periodic_cycle
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        self.src = SceneRegionCount(
            self.config, self.time_window, self.time_increment, 1
        )
        rospy.loginfo(
            "Creating a periodic cycle every %d minutes" % periodic_cycle
        )
        self.process = {
            roi: SpectralPoissonProcesses(
                window, increment, periodic_cycle
            ) for roi in self.regions.keys()
        }

    def learn_scene_patterns(self, start, end):
        rospy.loginfo("Updating scene processes for each region...")
        act_count_per_roi = self.src.count_scenes_per_region(start, end)
        if not self.src._is_scene_received:
            rospy.loginfo("No data received from db, return...")
            return
        for roi in self.process:
            rospy.loginfo(
                "Updating scene processes for region %s..." % roi
            )
            count_per_time = dict()
            if roi in act_count_per_roi and len(act_count_per_roi[roi]):
                count_per_time = act_count_per_roi[roi]
                self._update_scene_process(roi, count_per_time)

    def _update_scene_process(self, roi, count_per_time):
        ordered = sorted(count_per_time.keys())
        for start in ordered:
            count = count_per_time[start]
            self.process[roi].update(start, count)
            self._store(roi, start)
            # updating general starting time of the whole processes
            cond = self.process[roi]._init_time is not None
            cond = cond and (
                self._start_time is None or (
                    self._start_time > self.process[roi]._init_time
                )
            )
            if cond:
                self._start_time = self.process[roi]._init_time

    def _store(self, roi, start_time):
        self.process[roi]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "scene"
            }
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


if __name__ == '__main__':
    rospy.init_node("offline_scene_counter")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Fixed time window interval (in minute) for each Poisson distribution. Default is 10 minutes"
    )
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    parser.add_argument(
        "-c", dest="periodic_cycle", default="10080",
        help="Desired periodic cycle (in minute). Default is one week (10080 minutes)"
    )
    args = parser.parse_args()
    sc = SceneCounter(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle)
    )
    sc.load_from_db()
    start_time = raw_input("Start time 'year month day hour minute':")
    start_time = start_time.split(" ")
    start_time = datetime.datetime(
        int(start_time[0]), int(start_time[1]), int(start_time[2]),
        int(start_time[3]), int(start_time[4])
    )
    start_time = rospy.Time(time.mktime(start_time.timetuple()))
    end_time = raw_input("End time 'year month day hour minute':")
    end_time = end_time.split(" ")
    end_time = datetime.datetime(
        int(end_time[0]), int(end_time[1]), int(end_time[2]),
        int(end_time[3]), int(end_time[4])
    )
    end_time = rospy.Time(time.mktime(end_time.timetuple()))
    sc.learn_scene_patterns(start_time, end_time)
    rospy.spin()
