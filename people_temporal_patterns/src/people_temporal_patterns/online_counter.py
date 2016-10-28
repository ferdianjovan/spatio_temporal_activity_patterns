#!/usr/bin/env python

import copy
import time
import math
import rospy
import argparse
import datetime
import threading
from human_trajectory.msg import Trajectories
from spectral_processes.processes import SpectralPoissonProcesses
from region_observation.observation_proxy import RegionObservationProxy
from region_observation.util import create_line_string, is_intersected, get_soma_info
from people_temporal_patterns.offline_counter import PeopleCounter as OffPeopleCounter


class PeopleCounter(object):

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
        self._acquired = False
        # trajectories subscriber
        self.trajectories = list()
        self._traj_subs = rospy.Subscriber(
            rospy.get_param("~trajectory_topic", "/people_trajectory/trajectories/complete"),
            Trajectories, self._pt_cb, None, 10
        )
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, self.config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.time_window = window
        self.time_increment = increment
        self.periodic_cycle = periodic_cycle
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = {
            roi: SpectralPoissonProcesses(window, increment, periodic_cycle) for roi in self.regions.keys()
        }

    def request_stop_update(self):
        self._is_stop_requested = True
        self._traj_subs.unregister()

    def wait_to_stop(self):
        while not self._is_stopped:
            rospy.sleep(0.1)

    def request_continue_update(self):
        self._is_stop_requested = False
        self._traj_subs = rospy.Subscriber(
            rospy.get_param("~trajectory_topic", "/people_trajectory/trajectories/complete"),
            Trajectories, self._pt_cb, None, 10
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
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
            self.process[roi].retrieve_from_mongo(meta)

    def store_to_db(self):
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
            self.process[roi].store_to_mongo(meta)

    def _pt_cb(self, msg):
        if len(msg.trajectories) == 0:
            return
        while self._acquired:
            rospy.sleep(0.01)
        self._acquired = True
        self.trajectories.extend(msg.trajectories)
        self._acquired = False

    def continuous_update(self):
        self._is_stopped = False
        updating_region_time = rospy.Time.now()
        _is_updating_region = False
        _thread = None
        while not rospy.is_shutdown() and not self._is_stop_requested:
            now = rospy.Time.now()
            delta = (now - self._start_time)
            if delta > rospy.Duration(self.time_window*60+60):
                self.update()
            if datetime.datetime.fromtimestamp(now.secs).hour == 0:
                if not _is_updating_region:
                    updating_region_time = now
                    _thread = threading.Thread(target=self.update_regions)
                    _thread.start()
                    _is_updating_region = True
                    rospy.sleep(1)
            if _thread is not None and _thread.isAlive():
                if (now - updating_region_time).secs >= 3600:
                    updating_region_time = now
                    _thread.join(1)
                    if not _thread.isAlive():
                        _is_updating_region = False
                        _thread = None
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
            self._start_time, end_time, minute_increment=self.time_increment
        )
        temp = copy.deepcopy(self.trajectories)
        rospy.loginfo("Total trajectories counted so far is %d." % len(temp))

        used_trajectories = list()
        count_per_region = dict()
        for observation in region_observations:
            count = 0
            for trajectory in temp:
                points = [
                    [
                        pose.pose.position.x, pose.pose.position.y
                    ] for pose in trajectory.trajectory
                ]
                points = create_line_string(points)
                if is_intersected(self.regions[observation.region_id], points):
                    # it also must be within time boundaries
                    conditions = trajectory.end_time >= observation.start_from
                    # observation.until is secs.999999999999
                    conditions = conditions and trajectory.end_time <= observation.until
                    if conditions:
                        count += 1
                        if trajectory.end_time < self._start_time + rospy.Duration(self.time_increment*60):
                            used_trajectories.append(trajectory)
            if count > 0 or observation.duration.secs >= 59:
                count = self._extrapolate_count(observation.duration, count)
                if observation.region_id not in count_per_region.keys():
                    count_per_region[observation.region_id] = 0
                count_per_region[observation.region_id] += count
        # update and save observation for that time
        for roi, count in count_per_region.iteritems():
            self.process[roi].update(self._start_time, count)
            self._store(roi, self._start_time)
        self._start_time = self._start_time + rospy.Duration(self.time_increment*60)
        # remove trajectories that have been updated,
        # and update the current stored trajectories
        n = len(temp)
        temp = [i for i in temp if i not in used_trajectories]
        while self._acquired:
            rospy.sleep(0.01)
        self._acquired = True
        self.trajectories = temp + self.trajectories[n:]
        self._acquired = False

    def _store(self, roi, start_time):
        self.process[roi]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "people"
            }
        )

    def _extrapolate_count(self, duration, count):
        """ extrapolate the number of trajectories with specific upper_threshold.
            upper_threshold is to ceil how long the robot was in an area
            for one minute interval, if the robot was there for less than 20
            seconds, then it will be boosted to 20 seconds.
        """
        upper_threshold_duration = rospy.Duration(0, 0)
        while duration > upper_threshold_duration:
            upper_threshold_duration += rospy.Duration(
                self.time_increment * 20, 0
            )

        multiplier_estimator = 3600 / float(
            (60 / self.time_increment) * upper_threshold_duration.secs
        )
        # rospy.loginfo("Extrapolate count %d by %.2f" % (count, multiplier_estimator))
        return math.ceil(multiplier_estimator * count)

    def update_regions(self):
        regions, _ = get_soma_info(self.config)
        new_regions = {
            roi: region for roi, region in regions.iteritems() if roi not in self.regions
        }
        if new_regions == dict():
            rospy.loginfo("No new region is found, skipping next procedures...")
        else:
            rospy.loginfo(
                "New regions %s are found, proceeding to next procedures..." % str(
                    new_regions.keys()
                )
            )
            self.regions.update(new_regions)
            opc = OffPeopleCounter(
                self.config, self.time_window, self.time_increment, self.periodic_cycle, True
            )
            opc.construct_process_from_trajectory(
                self._start_time, rospy.Time.now(), new_regions.keys()
            )
            for roi in new_regions:
                self.process.update({roi: opc.process[roi]})


if __name__ == '__main__':
    rospy.init_node("online_people_counter")
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

    ppp = PeopleCounter(
        args.soma_config, int(args.time_window), int(args.time_increment),
        int(args.periodic_cycle)
    )
    ppp.load_from_db()
    ppp.continuous_update()
    rospy.spin()
