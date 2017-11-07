#!/usr/bin/env python

import rospy
import pymongo
import datetime
from geometry_msgs.msg import Pose
from region_observation.util import is_intersected
from mongodb_store.message_store import MessageStoreProxy
from region_observation.msg import Observation
from region_observation.util import robot_view_cone, get_soma_info


class OfflineRegionObservation(object):

    def __init__(self, config, time_increment=1, coll="region_observation"):
        # set minute increment of robot observation
        if 60 % time_increment != 0 or time_increment == 0:
            rospy.logwarn("The minute increment was not a factor of 60, setting it to 1...")
            time_increment = 1
        self.time_increment = time_increment
        # get map info
        rospy.loginfo("Obtaining soma information...")
        self.regions, self.soma_map = get_soma_info(config)
        self.soma_config = config
        # robot_pose in roslog
        self._pose_db = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).roslog.robot_pose
        # db for RegionObservation
        rospy.loginfo("Create collection db as %s..." % coll)
        self._db = MessageStoreProxy(collection=coll)

    def calculate_observation(self, start_time, end_time):
        while start_time + rospy.Duration(self.time_increment*60) < end_time:
            start_date = datetime.datetime.fromtimestamp(start_time.secs)
            start_date = datetime.datetime(
                start_date.year, start_date.month, start_date.day,
                start_date.hour, start_date.minute
            )
            mid_end = start_time + rospy.Duration(self.time_increment*60)
            end_date = datetime.datetime.fromtimestamp(mid_end.secs)
            end_date = datetime.datetime(
                end_date.year, end_date.month, end_date.day,
                end_date.hour, end_date.minute
            )
            durations, avg_robot_pose = self.get_duration(start_date, end_date)
            self.save_observation(durations, avg_robot_pose, start_time, mid_end)
            start_time = mid_end

    def save_observation(self, durations, avg_robot_pose, start_time, end_time):
        end_time = end_time - rospy.Duration(0, 1)
        for roi, duration in durations.iteritems():
            rospy.loginfo(
                "Save observation within %s and %s for region %s" % (
                    str(start_time.secs), str(end_time.secs), roi
                )
            )
            if duration > rospy.Duration(60):
                duration = rospy.Duration(60)
            msg = Observation(
                self.soma_map, self.soma_config, roi,
                start_time, end_time, duration
            )
            self._db.insert(
                msg, {"average_robot_pose": [
                    avg_robot_pose[0][0] / float(avg_robot_pose[1]),
                    avg_robot_pose[0][1] / float(avg_robot_pose[1]),
                ]}
            )

    def get_duration(self, start_date, end_date):
        roi_duration = dict()
        query = {
            "_id": {"$exists": "true"},
            "_meta.inserted_at": {"$gte": start_date, "$lt": end_date}
        }
        avg_robot_pose = [[0.0, 0.0], 0]
        for log in self._pose_db.find(query).sort("_meta.inserted_at", 1):
            pose = Pose()
            pose.position.x = log['position']['x']
            pose.position.y = log['position']['y']
            pose.position.z = log['position']['z']
            pose.orientation.x = log['orientation']['x']
            pose.orientation.y = log['orientation']['y']
            pose.orientation.z = log['orientation']['z']
            pose.orientation.w = log['orientation']['w']
            robot_sight, _ = robot_view_cone(pose, 0.0)
            avg_robot_pose = [
                [
                    pose.position.x + avg_robot_pose[0][0],
                    pose.position.y + avg_robot_pose[0][1],
                ],
                avg_robot_pose[1] + 1
            ]
            for roi, region in self.regions.iteritems():
                if is_intersected(robot_sight, region):
                    if roi not in roi_duration:
                        roi_duration[roi] = [log['_meta']['inserted_at'], log['_meta']['inserted_at']]
                    else:
                        roi_duration[roi][-1] = log['_meta']['inserted_at']
        duration = dict()
        for roi, [start, end] in roi_duration.iteritems():
            duration[roi] = rospy.Duration((end - start).total_seconds())
        return duration, avg_robot_pose
