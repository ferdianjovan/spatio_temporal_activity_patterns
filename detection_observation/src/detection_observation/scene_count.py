#!/usr/bin/env python

import time
import rospy
import argparse
import datetime
from detection_observation.msg import DetectionObservation
from simple_change_detector.msg import ChangeDetectionMsg
from mongodb_store.message_store import MessageStoreProxy
from detection_observation.detection_count import DetectionCountObservation
from region_observation.util import create_line_string, is_intersected


class SceneCountObservation(DetectionCountObservation):

    def __init__(
        self, config, increment=rospy.Duration(60),
        max_count_per_increment=1
    ):
        rospy.loginfo("Initiating Scene Region Counting...")
        super(SceneCountObservation, self).__init__(
            config, increment, max_count_per_increment
        )
        self._scene_db = MessageStoreProxy(collection="change_detection")

    def load_observation(self, start_time, end_time, roi=""):
        return super(SceneCountObservation, self).load_observation(
            start_time, end_time, roi, "scene"
        )

    def get_scenes_from_mongo(self, start_time, end_time):
        logs = self._scene_db.query(
            ChangeDetectionMsg._type,
            {"header.stamp.secs": {"$gte": start_time.secs, "$lt": end_time.secs}}
        )
        return [log[0] for log in logs]

    def store_scene_observation_per_time_increment(self, start_time, end_time):
        rospy.loginfo(
            "Storing scene observation from %s to %s" % (
                str(datetime.datetime.fromtimestamp(start_time.secs)),
                str(datetime.datetime.fromtimestamp(end_time.secs))
            )
        )
        start_calc = rospy.Time.now()
        mid_end = start_time + self.time_increment
        while mid_end <= end_time:
            scenes = self.get_scenes_from_mongo(start_time, mid_end)
            for roi, region in self.regions.iteritems():
                count = 0
                is_max_reached = False
                robot_pose = list()
                for scene in scenes:
                    for centroid in scene.object_centroids:
                        point = create_line_string([centroid.x, centroid.y])
                        if is_intersected(region, point):
                            count += 1
                            robot_pose.append([
                                scene.robot_pose.position.x,
                                scene.robot_pose.position.y
                            ])
                        if count >= self._max_count:
                            is_max_reached = True
                            break
                    if is_max_reached:
                        break
                count = min(self._max_count, count)
                if not count:
                    full_obs = self.obs_proxy.is_robot_present_all_time(
                        start_time, mid_end, roi=roi,
                        minute_increment=self.time_increment.secs/60
                    )
                    if full_obs:
                        robot_pose = self.obs_proxy.avg_robot_pose(
                            start_time, mid_end, roi=roi,
                            minute_increment=self.time_increment.secs/60
                        )
                if count or full_obs:
                    rospy.loginfo(
                        "Roi %s has %d detection at %s-%s" % (
                            roi, count, str(
                                datetime.datetime.fromtimestamp(start_time.secs)
                            ), str(
                                datetime.datetime.fromtimestamp(mid_end.secs)
                            )
                        )
                    )
                    msg = DetectionObservation(
                        self.map, self.config, roi, start_time,
                        (mid_end-rospy.Duration(0, 1)), count,
                        "scene", int(self._max_count)
                    )
                    if len(robot_pose):
                        self._db.insert(
                            msg, {
                                "robot_pose": [
                                    sum(zip(*robot_pose)[0]) / float(len(robot_pose)),
                                    sum(zip(*robot_pose)[1]) / float(len(robot_pose)),
                                ]
                            }
                        )
                    else:
                        self._db.insert(msg, {"robot_pose": []})
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_increment
        end_calc = rospy.Time.now()
        rospy.loginfo("Total calculation and storing time: %d" % (
            (end_calc - start_calc).secs
        ))


if __name__ == '__main__':
    rospy.init_node("scene_counter")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    parser.add_argument(
        "-c", dest="max_count", default="1",
        help="Maximum counting value per time increment. Default is 1"
    )
    args = parser.parse_args()
    src = SceneCountObservation(
        args.soma_config, increment=rospy.Duration(int(args.time_increment)*60),
        max_count_per_increment=float(args.max_count)
    )
    start_time = raw_input("Start time 'year month day hour minute':")
    start_time = start_time.split(" ")
    start_time = datetime.datetime(
        int(start_time[0]), int(start_time[1]), int(start_time[2]),
        int(start_time[3]), int(start_time[4])
    )
    start_time = rospy.Time(time.mktime(start_time.timetuple()))
    end_time = raw_input("End time 'year month day hour minute':")
    end_time = end_time.split(" ")
    end_time = datetime.datetime(
        int(end_time[0]), int(end_time[1]), int(end_time[2]),
        int(end_time[3]), int(end_time[4])
    )
    end_time = rospy.Time(time.mktime(end_time.timetuple()))
    src.store_scene_observation_per_time_increment(start_time, end_time)
