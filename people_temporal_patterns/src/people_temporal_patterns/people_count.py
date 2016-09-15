#!/usr/bin/env python

import time
import copy
import rospy
import datetime
from human_trajectory.trajectories import OfflineTrajectories
from region_observation.observation_proxy import RegionObservationProxy
from region_observation.util import create_line_string, is_intersected, get_soma_info


class RegionPeopleCount(object):

    def __init__(self, config, window=10, increment=1):
        self.time_window = window
        self.time_increment = increment
        # get regions and soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, config)
        # result from get_people_per_region
        self.current_count_per_region = {
            region_id: dict() for region_id in self.regions.keys()
        }

    def get_people_per_region(self, start_time, end_time):
        result = {region_id: dict() for region_id in self.regions.keys()}
        temp = datetime.datetime.fromtimestamp(start_time.secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        start_time = rospy.Time(time.mktime(temp.timetuple()))
        mid_end = start_time + rospy.Duration(self.time_window*60)
        while mid_end <= end_time:
            rospy.loginfo(
                "Obtaining region and trajectories between %s and %s" % (
                    start_time.secs, mid_end.secs
                )
            )
            count_per_region = self.get_people_per_region_within_time_window(
                start_time
            )
            for roi, count in count_per_region.iteritems():
                result[roi][start_time] = count
            start_time = start_time + rospy.Duration(self.time_increment*60)
            mid_end = start_time + rospy.Duration(self.time_window*60)
        self.current_count_per_region = copy.deepcopy(result)
        # must add self.plot to plot all region count
        return result

    def get_people_per_region_within_time_window(self, start_time):
        end_time = start_time + rospy.Duration(self.time_window*60)
        used_trajectories = dict()
        count_per_region = dict()
        total_observation_time = dict()
        region_observations = self.obs_proxy.load_msg(
            start_time, end_time, minute_increment=self.time_increment
        )
        # query all trajectories that starts or ends or
        # starts longer than the start_time + the time window
        query = {"$or": [
            {"end_time.secs": {
                "$gte": start_time.secs, "$lt": end_time.secs
            }},
            {"start_time.secs": {
                "$gte": start_time.secs, "$lt": end_time.secs

            }},
            {
                "start_time.secs": {"$lt": start_time.secs},
                "end_time.secs": {"$gte": end_time.secs}
            }
        ]}
        trajectories = OfflineTrajectories(query, size=10000)
        # get trajectory messages
        trajectories = [
            i.get_trajectory_message() for i in trajectories.traj.values()
        ]
        for observation in region_observations:
            count = 0
            for trajectory in trajectories:
                points = [
                    [
                        pose.pose.position.x, pose.pose.position.y
                    ] for pose in trajectory.trajectory
                ]
                points = create_line_string(points)
                # trajectory must intersect with any region
                if is_intersected(self.regions[observation.region_id], points):
                    if observation.region_id not in used_trajectories.keys():
                        used_trajectories[observation.region_id] = list()
                    if trajectory not in used_trajectories[observation.region_id]:
                        used_trajectories[observation.region_id].append(trajectory)
                        count += 1
            if observation.region_id not in total_observation_time:
                total_observation_time[observation.region_id] = rospy.Duration(0, 0)
            total_observation_time[observation.region_id] += observation.duration
            if count > 0 or observation.duration.secs >= 59:
                if observation.region_id not in count_per_region.keys():
                    count_per_region[observation.region_id] = list()
                count_per_region[observation.region_id].append(count)
        results = dict()
        for roi, counts in count_per_region.iteritems():
            if sum(counts) > 0 or len(counts) == (self.time_window / self.time_increment):
                divider = 4.0
                multiplier = 3.0
                threshold = self.time_window * 60
                substractor = (self.time_window * 60) / multiplier
                if sum(counts) > 0:
                    while total_observation_time[roi].secs <= threshold and divider > 1.0:
                        divider -= 1.0
                        threshold -= substractor
                # use multiplier, with uniform assumption
                results[roi] = sum(counts) * (multiplier / divider)
        return results
