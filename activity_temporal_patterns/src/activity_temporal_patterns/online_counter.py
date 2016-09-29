#!/usr/bin/env python

import copy
import time
import rospy
import datetime
from activity_simulation.msg import ActivitiesMsg
from activity_simulation.srv import ActivitiesSrv
from region_observation.util import get_soma_info
from poisson_processes.processes import PeriodicPoissonProcesses


class ActivityCounter(object):

    def __init__(
        self, config, available_activity_srv, activity_topic,
        window=10, increment=1, periodic_cycle=10080, coll="poisson_processes",
    ):
        rospy.loginfo("Starting activity processes...")
        temp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        self._start_time = rospy.Time(time.mktime(temp.timetuple()))
        self._acquired = False
        # activity subscriber
        self._uuids = list()
        self.activities = list()
        act_srv = rospy.ServiceProxy(available_activity_srv, ActivitiesSrv)
        act_srv.wait_for_service()
        self.available_activities = act_srv().activities
        rospy.loginfo("Known activities: %s" % str(self.available_activities))
        rospy.Subscriber(activity_topic, ActivitiesMsg, self._pt_cb, None, 10)
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = {
            roi: {
                act: PeriodicPoissonProcesses(window, increment, periodic_cycle) for act in self.available_activities
            } for roi in self.regions
        }

    def _pt_cb(self, msg):
        if len(msg.activities) == 0:
            return
        while self._acquired:
            rospy.sleep(0.01)
        self._acquired = True
        for act in msg.activities:
            if act.uuid not in self._uuids:
                self._uuids.append(act.uuid)
                self.activities.append(act)
        self._acquired = False

    # scale might not be needed
    def retrieve_from_to(self, start_time, end_time, scale=False):
        grand_result = dict()
        for roi, acts in self.process.iteritems():
            result = dict()
            for act, poisson in acts.iteritems():
                result.update(
                    # use upper confidence rate value
                    {act: poisson.retrieve(
                        start_time, end_time, use_upper_confidence=True, scale=scale
                    )}
                )
            grand_result.update({roi: result})
        return grand_result

    def load_from_db(self):
        for roi in self.process:
            for act in self.process[roi]:
                meta = {
                    "soma_map": self.map, "soma_config": self.config,
                    "region_id": roi, "type": "activity", "activity": act
                }
                self.process[roi][act].retrieve_from_mongo(meta)

    def store_to_db(self):
        for roi in self.process:
            for act in self.process[roi]:
                meta = {
                    "soma_map": self.map, "soma_config": self.config,
                    "region_id": roi, "type": "activity", "activity": act
                }
                self.process[roi][act].store_to_mongo(meta)

    def continuous_update(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            if (current_time - self._start_time) > self.time_window:
                rospy.loginfo("Updating activity processes for each region...")
                self.update(current_time)
            rospy.sleep(1)

    def update(self, current_time):
        checked_activities = list()
        count_per_act_per_region = dict()
        temp = copy.deepcopy(self.activities)
        rospy.loginfo("Total activities so far is %d." % len(temp))
        count_per_act = {act: 0 for act in self.available_activities}

        for activity in temp:
            # it also must be within time boundaries
            # observation.until is secs.999999999999
            end_term = activity.end >= self._start_time and activity.end <= current_time
            start_term = activity.start >= self._start_time and activity.start <= current_time
            # Need to deal with ongoing activities that were started before self._start_time
            # unfinished_term is wrongly implemented, ignore atm!
            unfinished_term = activity.start < self._start_time and activity.end > current_time
            if end_term or start_term or unfinished_term:
                if activity.region not in count_per_act_per_region:
                    count_per_act_per_region[activity.region] = {
                        act: 0 for act in self.available_activities
                    }
                count_per_act_per_region[activity.region][activity.activity] += 1
                if activity.end < self._start_time + self.time_increment:
                    checked_activities.append(activity)

        # update and save observation for that time
        for roi, count_per_act in count_per_act_per_region.iteritems():
            for act, count in count_per_act.iteritems():
                self.process[roi][act].update(self._start_time, count)
                self._store(roi, act, self._start_time)
        self._start_time = self._start_time + self.time_increment
        # remove trajectories that have been updated,
        # and update the current stored trajectories
        n = len(temp)
        temp = [i for i in temp if i not in checked_activities]
        while self._acquired:
            rospy.sleep(0.01)
        self._acquired = True
        self.activities = temp + self.activities[n:]
        self._acquired = False

    def _store(self, roi, act, start_time):
        self.process[roi][act]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "activity", "activity": act
            }
        )
