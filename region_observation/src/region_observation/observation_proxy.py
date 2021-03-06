#!/usr/bin/env python

import time
import rospy
import datetime
from region_observation.msg import Observation
from mongodb_store.message_store import MessageStoreProxy


class RegionObservationProxy(object):

    def __init__(
        self, soma_map, soma_config, coll="region_observation"
    ):
        rospy.loginfo("Initializing region observation proxy...")
        self.soma_map = soma_map
        self.soma_config = soma_config
        rospy.loginfo(
            "Soma map is %s with the configuration %s" %
            (soma_map, soma_config)
        )
        self._db = MessageStoreProxy(collection=coll)

    def load_dict(self, start_time, end_time, roi="", minute_increment=1):
        # [roi[month[day[hour[minute:duration]]]]]
        logs = self.load_msg(start_time, end_time, roi, minute_increment)
        roi_observation = dict()
        total_observation = rospy.Duration(0, 0)
        for log in logs:
            start = datetime.datetime.fromtimestamp(log.start_from.secs)
            end = log.until + rospy.Duration(0, 1)
            end = datetime.datetime.fromtimestamp(end.secs)
            if end.minute - start.minute == minute_increment:
                if log.region_id not in roi_observation:
                    roi_observation[log.region_id] = dict()
                if start.month not in roi_observation[log.region_id]:
                    roi_observation[log.region_id][start.month] = dict()
                if start.day not in roi_observation[log.region_id][start.month]:
                    roi_observation[log.region_id][start.month][start.day] = dict()
                if start.hour not in roi_observation[log.region_id][start.month][start.day]:
                    roi_observation[log.region_id][start.month][start.day][start.hour] = dict()
                key = "%s-%s" % (start.minute, end.minute)
                roi_observation[log.region_id][start.month][start.day][start.hour][key] = log.duration
                total_observation += log.duration
        return roi_observation, total_observation

    def load_msg(self, start_time, end_time, roi="", minute_increment=1):
        new_start = datetime.datetime.fromtimestamp(start_time.secs)
        new_start = datetime.datetime(
            new_start.year, new_start.month, new_start.day, new_start.hour,
            new_start.minute
        )
        start_time = rospy.Time(time.mktime(new_start.timetuple()))
        new_end = datetime.datetime.fromtimestamp(end_time.secs)
        new_end = datetime.datetime(
            new_end.year, new_end.month, new_end.day, new_end.hour,
            new_end.minute
        )
        end_time = rospy.Time(time.mktime(new_end.timetuple()))
        query = {
            "soma": self.soma_map, "soma_config": self.soma_config,
            "start_from.secs": {"$gte": start_time.secs, "$lt": end_time.secs}
        }
        if roi != "":
            query.update({"region_id": roi})
        logs = self._db.query(Observation._type, query)
        # rospy.loginfo("Between %s and %s, got %d region observation entries..." % (
        #     str(start_time.secs), str(end_time.secs), len(logs)
        # ))
        return [log[0] for log in logs]
