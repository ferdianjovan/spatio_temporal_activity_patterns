#!/usr/bin/env python

import copy
import time
import rospy
import datetime
from region_observation.util import get_soma_info
from spectral_processes.processes import SpectralPoissonProcesses
from activity_temporal_patterns.activity_count import DailyActivityRegionCount


class ActivityCounter(object):

    def __init__(
        self, config, window=10, increment=1, periodic_cycle=10080
    ):
        rospy.loginfo("Starting activity processes...")
        self._start_time = None
        self._max_activity_types = 15
        self._last_learning_date = datetime.date.fromtimestamp(rospy.Time.now().secs)
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.periodic_cycle = periodic_cycle
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = {roi: dict() for roi in self.regions}

    def learn_activity_patterns(self, date):
        darc = DailyActivityRegionCount(
            date, self.config, self.time_window, self.time_increment
        )
        act_count_per_roi = darc.activity_count_per_roi
        if darc._is_activity_received:
            for roi in self.process:
                rospy.loginfo("Updating activity processes for region %s..." % roi)
                count_per_time = dict()
                if roi in act_count_per_roi:
                    count_per_time = act_count_per_roi[roi]
                if len(count_per_time):
                    ordered = sorted(count_per_time.keys())
                    for start in ordered:
                        count_per_act = count_per_time[start]
                        for act_ind, count in enumerate(count_per_act):
                            if act_ind not in self.process[roi]:
                                self.process[roi][act_ind] = SpectralPoissonProcesses(
                                    self.time_window.secs/60,
                                    self.time_increment.secs/60, self.periodic_cycle
                                )
                            self.process[roi][act_ind].update(start, count)
                            self._store(roi, act_ind, start)
                            # updating general starting time of the whole processes
                            cond = self.process[roi][act_ind]._init_time is not None
                            cond = cond and (
                                self._start_time is None or self._start_time > self.process[roi][act_ind]._init_time
                            )
                            if cond:
                                self._start_time = self.process[roi][act_ind]._init_time
        return darc._is_activity_received

    def _store(self, roi, act, start_time):
        self.process[roi][act]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "activity", "activity": act
            }
        )

    def continuous_update(self):
        while not rospy.is_shutdown():
            cur_date = datetime.date.fromtimestamp(rospy.Time.now().secs)
            if (cur_date - self._last_learning_date) >= datetime.timedelta(days=1):
                rospy.loginfo("Updating activity processes for each region...")
                is_successful = self.learn_activity_patterns(self._last_learning_date)
                if is_successful or (
                    cur_date - self._last_learning_date
                ) >= datetime.timedelta(days=2):
                    self._last_learning_date += datetime.timedelta(days=1)
            rospy.sleep(3600)

    # scale might not be needed
    def retrieve_from_to(self, start_time, end_time, use_upper_confidence=False, scale=False):
        grand_result = dict()
        for roi, acts in self.process.iteritems():
            result = dict()
            for act, poisson in acts.iteritems():
                result.update(
                    # use upper confidence rate value
                    {act: poisson.retrieve(
                        start_time, end_time,
                        use_upper_confidence=use_upper_confidence, scale=scale
                    )}
                )
            grand_result.update({roi: result})
        return grand_result

    def load_from_db(self):
        rospy.loginfo("Retrieving activity processes from database. It may take a while...")
        for roi in self.process:
            rospy.loginfo("Retrieving activity processes for region %s" % roi)
            for act in range(self._max_activity_types):
                rospy.loginfo("Retrieving activity processes for act %d" % act)
                meta = {
                    "soma_map": self.map, "soma_config": self.config,
                    "region_id": roi, "type": "activity", "activity": act
                }
                self.process[roi][act] = SpectralPoissonProcesses(
                    self.time_window.secs/60,
                    self.time_increment.secs/60, self.periodic_cycle
                )
                is_retrieved = self.process[roi][act].retrieve_from_mongo(meta)
                if not is_retrieved:
                    break

    def store_to_db(self):
        for roi in self.process:
            for act in self.process[roi]:
                meta = {
                    "soma_map": self.map, "soma_config": self.config,
                    "region_id": roi, "type": "activity", "activity": act
                }
                self.process[roi][act].store_to_mongo(meta)


if __name__ == '__main__':
    rospy.init_node("activity_counter")
    ac = ActivityCounter("ferdi_test", periodic_cycle=60)
    ac._last_learning_date = datetime.date(2016, 10, 27)
    ac.load_from_db()
    ac.continuous_update()
    rospy.spin()
