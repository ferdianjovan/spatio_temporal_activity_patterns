#!/usr/bin/env python

import time
import copy
import rospy
import datetime
import numpy as np
from activity_data.msg import HumanActivities
from region_observation.util import get_soma_info
from mongodb_store.message_store import MessageStoreProxy
from region_observation.util import create_line_string, is_intersected
from region_observation.observation_proxy import RegionObservationProxy


class ActivityRegionCount(object):

    def __init__(
        self, config, window=rospy.Duration(600), increment=rospy.Duration(60)
    ):
        self._total_activities = 0
        self._start_time = rospy.Time.now()
        self.time_window = window
        self.time_increment = increment
        self._is_activity_received = True
        # get regions and soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, config)
        # activity related
        self._db = MessageStoreProxy(
            collection=rospy.get_param(
                "~activity_collection", "activity_learning"
            )
        )

    def count_activities_per_region(self):
        rospy.loginfo("Counting activity observation...")
        activities_per_roi = self.get_activities_per_region(
            self.get_activities_from_mongo()
        )
        backup = copy.deepcopy(activities_per_roi)
        activity_count_per_roi = dict()
        if self._is_activity_received:
            for roi, _ in self.regions.iteritems():
                rospy.loginfo(
                    "Calculating number of activities in region %s" % roi
                )
                activities = list()
                if roi in activities_per_roi:
                    activities = activities_per_roi[roi]
                activity_count_per_roi[
                    roi
                ] = self.count_activities_within_time_window(activities, roi)
        # if acts obtained then prev_seconds is dur(0), otherwise not dur(0)
        return activity_count_per_roi, backup

    def update_activities_to_mongo(self, activities, restart_start_time=True):
        for act in activities:
            act.temporal = True
            self._db.update(act, message_query={"uuid": act.uuid})
        if restart_start_time:
            self._start_time = rospy.Time.now()

    def get_activities_from_mongo(self, is_temporal=False):
        rospy.loginfo(
            "Getting activity details from %s collection" % rospy.get_param(
                "~activity_collection", "activity_learning"
            )
        )
        logs = self._db.query(
            HumanActivities._type,
            {"activity": True, "temporal": is_temporal}
        )
        self._is_activity_received = len(logs) > 0
        # projection_query={"robot_data": 0, "skeleton_data": 0}
        activities = list()
        unprocessed_data = False
        for log in logs:
            if log[0].start_time.secs == 0:
                times = log[0].time.split(":")
                date = log[0].date.split("-")
                start_time = datetime.datetime(
                    int(date[0]), int(date[1]), int(date[2]),
                    int(times[0]), int(times[1]), int(times[2])
                )
                start_time = rospy.Time(time.mktime(start_time.timetuple()))
                log[0].start_time = start_time
                log[0].end_time = start_time + rospy.Duration(60)
                if self._start_time > start_time:
                    self._start_time = start_time
                    unprocessed_data = True
            elif log[0].start_time.secs > 0 and self._start_time > log[0].start_time:
                self._start_time = log[0].start_time
                unprocessed_data = True
            activities.append(log[0])
            act_len = len(log[0].topics)
            if act_len > self._total_activities:
                self._total_activities = act_len
        if unprocessed_data:
            rospy.loginfo(
                "There are unprocessed activities since %s." % (
                    datetime.datetime.fromtimestamp(self._start_time.secs)
                )
            )
        return activities

    def get_activities_per_region(self, activities):
        activity_group_regions = dict()
        activities_not_in_regions = list()
        for roi, region in self.regions.iteritems():
            for activity in activities:
                point = create_line_string(
                    [activity.map_point.x, activity.map_point.y]
                )
                if is_intersected(region, point):
                    if roi not in activity_group_regions:
                        activity_group_regions[roi] = list()
                    activity_group_regions[roi].append(activity)
                    if activity not in activities_not_in_regions:
                        activities_not_in_regions.append(activity)
        activities_not_in_regions = [
            i for i in activities if i not in activities_not_in_regions
        ]
        self.update_activities_to_mongo(activities_not_in_regions, False)
        for roi, val in activity_group_regions.iteritems():
            rospy.loginfo(
                "%d activities are found in region %s" % (len(val), roi)
            )
        return activity_group_regions

    def _get_activities_within_time_window(self, activities, start_time):
        result = list()
        past_activities = list()
        end_time = start_time + self.time_window
        for act in activities:
            if start_time > act.end_time:
                past_activities.append(act)
                continue
            if act.end_time < act.start_time:
                rospy.logwarn(
                    "Observed activity %s ended before it began" % act.uuid
                )
                continue
            end_cond = act.end_time >= start_time and act.end_time < end_time
            start_cond = act.start_time >= start_time and act.start_time < end_time
            between_cond = act.start_time < start_time and act.end_time >= end_time
            if end_cond or start_cond or between_cond:
                result.append(act)
        return result, past_activities

    def count_activities_within_time_window(self, activities, roi=""):
        counts = dict()
        past_activities = list()
        start_time = self._start_time
        end_time = rospy.Time.now()
        mid_end = start_time + self.time_window
        while mid_end <= end_time:
            tmp_activities = list()
            count = 0
            if len(activities) > 0:
                tmp_activities, past_act = self._get_activities_within_time_window(
                    activities, start_time
                )
                tmp_activities = np.array(
                    [act.topics for act in tmp_activities], dtype="float64"
                )
                tmp_activities = [i/np.linalg.norm(i) for i in tmp_activities]
                past_activities += past_act
                activities = [i for i in activities if i not in past_activities]
                count = sum(tmp_activities)
            exist_act = (
                len(tmp_activities) > 0 and True not in np.isnan(count)
            )
            if exist_act:
                counts.update({start_time: count})
            else:
                full_obs = self.obs_proxy.is_robot_present_all_time(
                    start_time, mid_end, roi=roi,
                    minute_increment=self.time_increment.secs/60
                )
                if full_obs:
                    count = [0.0 for i in range(self._total_activities)]
                    counts.update({start_time: count})
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_window
        return counts


if __name__ == '__main__':
    rospy.init_node("activity_region_count_test")
    ActivityRegionCount("poisson_activity")
