#!/usr/bin/env python

import time
import yaml
import rospy
import datetime
import numpy as np
import cPickle as pickle
from skeleton_tracker.msg import SkeletonComplete
from mongodb_store.message_store import MessageStoreProxy
from region_observation.observation_proxy import RegionObservationProxy
from region_observation.util import create_line_string, is_intersected, get_soma_info


class DailyActivityRegionCount(object):

    def __init__(
        self, date, config, window=rospy.Duration(600),
        increment=rospy.Duration(60)
    ):
        self.date = date
        self.time_window = window
        self.time_increment = increment
        self._is_activity_received = True
        # get regions and soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, config)
        # activity related
        self._db = MessageStoreProxy(
            collection=rospy.get_param("~skeleton_collection", "people_skeleton")
        )
        self._activity_path = rospy.get_param(
            "~activity_learning_path", "/home/fxj345/SkeletonDataset/Learning/accumulate_data"
        )
        skeletons = self._get_skeletons_from_db(date)
        activities = self._get_activities_from_file(date)
        activities_per_roi = self.group_per_region(activities, skeletons)
        self.activity_count_per_roi = dict()
        for roi, activities in activities_per_roi.iteritems():
            rospy.loginfo("Calculating number of activities in region %s" % roi)
            self.activity_count_per_roi[
                roi
            ] = self._group_activities_within_region(activities, roi)

    def _get_skeletons_from_db(self, date):
        rospy.loginfo("Getting skeleton details for date: %s" % str(self.date))
        skeletons = dict()
        query = {
            "date": str(date.year)+"-"+str(date.month)+"-"+str(date.day)
        }
        logs = self._db.query(
            SkeletonComplete._type, query, {},
            projection_query={"robot_data":0, "skeleton_data":0}
        )
        if len(logs) > 0:
            skeletons = {
                log[0].uuid: (
                    log[0].human_map_point, log[0].start_time,
                    log[0].end_time
                ) for log in logs
            }
            skeletons.update(skeletons)
        rospy.loginfo("Number of skeletons obtained: %d" % len(skeletons))
        return skeletons

    def _get_activities_from_file(self, date):
        rospy.loginfo("Getting activity details for date: %s" % str(self.date))
        activities = dict()
        date = str(date.year)+"-"+str(date.month)+"-"+str(date.day)
        try:
            uuids = pickle.load(open(self._activity_path+"/"+date+"/list_of_uuids.p", "r"))
            activities = yaml.load(
                open(
                    self._activity_path+"/"+date+"/oLDA/gamma.dat", "r"
                )
            ).split(" ")
            activities = np.array(activities, dtype="float64")
            num_of_activities = activities.size/len(uuids)
            activities = activities.reshape(activities.size/num_of_activities, num_of_activities)
            activities = [i/np.linalg.norm(i) for i in activities]
            activities = {i[0]: i[1] for i in zip(uuids, activities)}
        except IOError as e:
            self._is_activity_received = False
            rospy.logerr("Some files do not exist, giving flag...")
        rospy.loginfo("Number of activities: %d" % len(activities))
        return activities

    def group_per_region(self, activities, skeletons):
        activity_group_regions = dict()
        for roi, region in self.regions.iteritems():
            for uuid, act in activities.iteritems():
                skel = skeletons[uuid]
                point = create_line_string([skel[0].x, skel[0].y])
                if is_intersected(region, point):
                    if roi not in activity_group_regions:
                        activity_group_regions[roi] = dict()
                    activity_group_regions[roi].update({uuid: (act, skel)})
        return activity_group_regions

    def _get_activities_within_time_window(self, activities, start_time):
        result = dict()
        end_time = start_time + self.time_window
        for uuid, act in activities.iteritems():
            end_cond = act[1][2] >= start_time and act[1][2] < end_time
            start_cond = act[1][1] >= start_time and act[1][1] < end_time
            between_cond = act[1][1] < start_time and act[1][2] >= end_time
            if end_cond or start_cond or between_cond:
                result.update({uuid: act})
        return result

    def _group_activities_within_region(self, activities, roi=""):
        counts = dict()
        start_time = rospy.Time(time.mktime(self.date.timetuple()))
        end_time = self.date + datetime.timedelta(days=1)
        end_time = rospy.Time(time.mktime(end_time.timetuple()))
        mid_end = start_time + self.time_window
        while mid_end <= end_time:
            tmp_activities = self._get_activities_within_time_window(
                activities, start_time
            )
            tmp_activities = [act[0] for act in tmp_activities.values()]
            count = sum(tmp_activities)
            region_observations = self.obs_proxy.load_msg(
                start_time, mid_end, minute_increment=self.time_increment.secs/60, roi=roi
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
    rospy.init_node("test")
    DailyActivityRegionCount(
        datetime.datetime(2016, 10, 28, 0, 0).date(), "ferdi_test"
    )
