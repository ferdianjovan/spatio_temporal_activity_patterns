#!/usr/bin/env python

import rospy
from region_observation.util import get_soma_info
from spectral_processes.processes import SpectralPoissonProcesses
from activity_temporal_patterns.activity_count import ActivityRegionCount


class ActivityCounter(object):

    def __init__(
        self, config, window=10, increment=1, periodic_cycle=10080,
        update_every=60
    ):
        rospy.loginfo("Starting activity processes...")
        self._start_time = None
        self._max_activity_types = 15
        # for pause feature
        self._is_stop_requested = False
        self._is_stopped = False
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        # for each roi create PoissonProcesses
        rospy.loginfo(
            "Time window is %d minute with increment %d minute" % (
                window, increment
            )
        )
        rospy.loginfo("Updating model every %d minute" % update_every)
        self.periodic_cycle = periodic_cycle
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        self.update_cycle = rospy.Duration(update_every*60)
        self.arc = ActivityRegionCount(
            self.config, self.time_window, self.time_increment
        )
        rospy.loginfo(
            "Creating a periodic cycle every %d minutes" % periodic_cycle
        )
        self.process = {roi: dict() for roi in self.regions}
        rospy.loginfo("Continuously observing activities...")
        rospy.Timer(self.update_cycle, self.learn_activity_patterns)

    def request_stop_update(self):
        self._is_stop_requested = True
        while not self._is_stopped:
            rospy.sleep(0.1)

    def request_continue_update(self):
        self._is_stop_requested = False

    def learn_activity_patterns(self, event):
        rospy.loginfo("Updating activity processes for each region...")
        if self._is_stop_requested:
            return
        act_count_per_roi, activities_per_roi = self.arc.count_activities_per_region()
        if self.arc._is_activity_received:
            for roi in self.process:
                if self._is_stop_requested:
                    break
                rospy.loginfo(
                    "Updating activity processes for region %s..." % roi
                )
                count_per_time = dict()
                if roi in act_count_per_roi and len(act_count_per_roi[roi]):
                    count_per_time = act_count_per_roi[roi]
                    self._update_activity_process(roi, count_per_time)
                    if roi in activities_per_roi:
                        self.arc.update_activities_to_mongo(
                            activities_per_roi[roi]
                        )

    def _update_activity_process(self, roi, count_per_time):
        ordered = sorted(count_per_time.keys())
        for start in ordered:
            if self._is_stop_requested:
                break
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
                    self._start_time is None or (
                        self._start_time > self.process[roi][act_ind]._init_time
                    )
                )
                if cond:
                    self._start_time = self.process[roi][act_ind]._init_time

    def _store(self, roi, act, start_time):
        self.process[roi][act]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "activity", "activity": act
            }
        )

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
    ac = ActivityCounter("poisson_activity", periodic_cycle=60, update_every=10)
    ac.load_from_db()
    ac.continuous_update()
    rospy.spin()
