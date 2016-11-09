#!/usr/bin/env python

import rospy
import numpy as np
from activity_data.msg import HumanActivities
from mongodb_store.message_store import MessageStoreProxy
from region_observation.observation_proxy import RegionObservationProxy
from region_observation.util import create_line_string, is_intersected, get_soma_info


class ActivityRegionCount(object):

    def __init__(
        self, config, window=rospy.Duration(600), increment=rospy.Duration(60)
    ):
        self.time_window = window
        self.time_increment = increment
        self._previous_seconds = rospy.Duration(0)
        self._is_activity_received = True
        # get regions and soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, config)
        # activity related
        self._db = MessageStoreProxy(
            collection=rospy.get_param("~activity_collection", "activity_learning")
        )

    def get_activities_per_region_for_last(self, seconds=rospy.Duration(3600)):
        rospy.loginfo(
            "Obtaining activity observation from the last %d seconds" % seconds.secs
        )
        activities_per_roi = self.get_activities_per_region(
            self.get_activities_from_mongo()
        )
        activity_count_per_roi = dict()
        if self._is_activity_received:
            for roi, activities in activities_per_roi.iteritems():
                rospy.loginfo("Calculating number of activities in region %s" % roi)
                self.activity_count_per_roi[
                    roi
                ] = self.group_activities_within_region(
                    activities, self._previous_seconds+seconds, roi
                )
            self._previous_seconds = rospy.Duration(0)
        else:
            self._previous_seconds += seconds
        # if acts obtained then prev_seconds is dur(0), otherwise not dur(0)
        return activity_count_per_roi

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
        return [log[0] for log in logs]

    def get_activities_per_region(self, activities):
        activity_group_regions = dict()
        for roi, region in self.regions.iteritems():
            for activity in activities:
                point = create_line_string(
                    [activity.map_point.x, activity.map_point.y]
                )
                if is_intersected(region, point):
                    if roi not in activity_group_regions:
                        activity_group_regions[roi] = list()
                    activity_group_regions[roi].append(activity)
        return activity_group_regions

    def _get_activities_within_time_window(self, activities, start_time):
        result = list()
        end_time = start_time + self.time_window
        for act in activities:
            assert act.end_time < act.start_time, "Skeleton ends before it starts!"
            end_cond = act.end_time >= start_time and act.end_time < end_time
            start_cond = act.start_time >= start_time and act.start_time < end_time
            between_cond = act.start_time < start_time and act.end_time >= end_time
            if end_cond or start_cond or between_cond:
                result.append(act)
        return result

    def group_activities_within_time_window(self, activities, seconds, roi=""):
        counts = dict()
        end_time = rospy.Time.now()
        start_time = end_time - seconds
        mid_end = start_time + self.time_window
        while mid_end <= end_time:
            tmp_activities = self._get_activities_within_time_window(
                activities, start_time
            )
            tmp_activities = np.array(
                [act.topics for act in tmp_activities], dtype="float64"
            )
            tmp_activities = [i/np.linalg.norm(i) for i in tmp_activities]
            count = sum(tmp_activities)
            region_observations = self.obs_proxy.load_msg(
                start_time, mid_end, roi=roi,
                minute_increment=self.time_increment.secs/60
            )
            total_observation_time = rospy.Duration(0, 0)
            for obs in region_observations:
                total_observation_time += obs.duration
            if len(tmp_activities) > 0 or len(region_observations) == (
                self.time_window.secs / self.time_increment.secs
            ):
                tmp = total_observation_time.secs / float(self.time_window.secs)
                if tmp <= 0.333:
                    count *= 3
                elif tmp > 0.333 and tmp <= 0.6777:
                    count *= 1.5
                counts.update({start_time: count})
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_window
        return counts


if __name__ == '__main__':
    rospy.init_node("activity_region_count_test")
    ActivityRegionCount("poisson_activity")
