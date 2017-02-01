#!/usr/bin/env python

import rospy
from region_observation.util import get_soma_info
from detection_observation.msg import DetectionObservation
from mongodb_store.message_store import MessageStoreProxy
from region_observation.observation_proxy import RegionObservationProxy


class DetectionCountObservation(object):

    def __init__(
        self, config, increment=rospy.Duration(60),
        max_count_per_increment=float("inf")
    ):
        rospy.loginfo("Initiating Detection Region Counting...")
        self._max_count = max_count_per_increment
        self.time_increment = increment
        # get regions and soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        self.obs_proxy = RegionObservationProxy(self.map, config)
        # db
        self._db = MessageStoreProxy(collection="detection_observation")

    def load_observation(self, start_time, end_time, roi="", detection_type=""):
        start_time = start_time.secs / 60
        start_time = rospy.Time(start_time * 60)
        end_time = end_time.secs / 60
        end_time = rospy.Time(end_time * 60)
        query = {
            "soma": self.map, "soma_config": self.config, "max_value": self._max_count,
            "start_time.secs": {"$gte": start_time.secs, "$lt": end_time.secs}
        }
        if roi != "":
            query.update({"region_id": roi})
        if detection_type != "":
            query.update({"detection_type": detection_type})
        logs = self._db.query(
            DetectionObservation._type, message_query=query,
            sort_query=[("start_time.secs", 1)]
        )
        return [log[0] for log in logs]
