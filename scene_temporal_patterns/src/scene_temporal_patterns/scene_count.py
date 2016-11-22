#!/usr/bin/env python

import rospy
from region_observation.util import get_soma_info
from simple_change_detector.msg import ChangeDetectionMsg
from mongodb_store.message_store import MessageStoreProxy
from region_observation.util import create_line_string, is_intersected
from region_observation.observation_proxy import RegionObservationProxy


class SceneRegionCount(object):

    def __init__(
        self, config, window=rospy.Duration(600),
        increment=rospy.Duration(60), max_count_per_increment=float("inf")
    ):
        self._max_count = max_count_per_increment
        self.time_window = window
        self.time_increment = increment
        self._is_scene_received = True
        # get regions and soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, config)
        # scene related
        self._db = MessageStoreProxy(collection="change_detection")

    def count_scenes_per_region(self, start_time, end_time):
        rospy.loginfo("Counting scene observation...")
        scenes_per_roi = self.get_scenes_per_region(
            self.get_scenes_from_mongo(start_time, end_time)
        )
        scene_count_per_roi = dict()
        if self._is_scene_received:
            for roi, _ in self.regions.iteritems():
                rospy.loginfo(
                    "Calculating number of scenes in region %s" % roi
                )
                scenes = list()
                if roi in scenes_per_roi:
                    scenes = scenes_per_roi[roi]
                scene_count_per_roi[
                    roi
                ] = self.count_scenes_within_time_window(scenes, start_time, end_time, roi)
        # if acts obtained then prev_seconds is dur(0), otherwise not dur(0)
        return scene_count_per_roi

    def get_scenes_from_mongo(self, start_time, end_time):
        rospy.loginfo(
            "Getting scene details from change_detection collection"
        )
        logs = self._db.query(
            ChangeDetectionMsg._type,
            {
                "header.stamp.secs": {"$gte": start_time.secs},
                "header.stamp.secs": {"$lt": end_time.secs}
            }
        )
        self._is_scene_received = len(logs) > 0
        return [log[0] for log in logs]

    def get_scenes_per_region(self, scenes):
        scene_group_regions = dict()
        for roi, region in self.regions.iteritems():
            for scene in scenes:
                for centroid in scene.object_centroids:
                    point = create_line_string([centroid.x, centroid.y])
                    if is_intersected(region, point):
                        if roi not in scene_group_regions:
                            scene_group_regions[roi] = list()
                        scene_group_regions[roi].append(scene.header.stamp)
        return scene_group_regions

    def _get_scenes_within_time_window(self, scenes, start_time):
        end_time = start_time + self.time_window
        result = [1 for act in scenes if start_time >= act and act < end_time]
        return min(self._max_count, sum(result))

    def count_scenes_within_time_window(self, scenes, start_time, end_time, roi=""):
        counts = dict()
        mid_end = start_time + self.time_window
        while mid_end <= end_time:
            count = 0
            if len(scenes) > 0:
                count = self._get_scenes_within_time_window(scenes, start_time)
            region_observations = self.obs_proxy.load_msg(
                start_time, mid_end, roi=roi,
                minute_increment=self.time_increment.secs/60
            )
            total_observation_time = rospy.Duration(0, 0)
            for obs in region_observations:
                total_observation_time += obs.duration
            full_obs = len(region_observations) == (
                self.time_window.secs / self.time_increment.secs
            )
            if count or full_obs:
                counts.update({start_time: count})
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_window
        return counts
