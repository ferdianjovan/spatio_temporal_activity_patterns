#!/usr/bin/env python

import time
import rospy
import argparse
import datetime
from detection_observation.msg import DetectionObservation
from human_trajectory.trajectories import OfflineTrajectories
from detection_observation.detection_count import DetectionCountObservation
from region_observation.util import create_line_string, is_intersected


class TrajectoryCountObservation(DetectionCountObservation):

    def __init__(
        self, config, increment=rospy.Duration(60),
        max_count_per_increment=1
    ):
        rospy.loginfo("Initiating Trajectory Region Counting...")
        super(TrajectoryCountObservation, self).__init__(
            config, increment, max_count_per_increment
        )

    def load_observation(self, start_time, end_time, roi=""):
        return super(TrajectoryCountObservation, self).load_observation(
            start_time, end_time, roi, "trajectory"
        )

    def get_trajectory_from_mongo(self, start_time, end_time):
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
        trajectory_db = OfflineTrajectories(query, size=10000)
        logs = [traj.get_trajectory_message() for traj in trajectory_db.traj.values()]
        return logs

    def store_trajectory_observation_per_time_increment(self, start_time, end_time):
        rospy.loginfo(
            "Storing trajectory observation from %s to %s" % (
                str(datetime.datetime.fromtimestamp(start_time.secs)),
                str(datetime.datetime.fromtimestamp(end_time.secs))
            )
        )
        start_calc = rospy.Time.now()
        mid_end = start_time + self.time_increment
        while mid_end <= end_time:
            for roi, region in self.regions.iteritems():
                count = 0
                trajectories = self.get_trajectory_from_mongo(start_time, mid_end)
                for trajectory in trajectories:
                    points = [
                        [
                            pose.pose.position.x, pose.pose.position.y
                        ] for pose in trajectory.trajectory
                    ]
                    points = create_line_string(points)
                    if is_intersected(region, points):
                        count += 1
                    if count >= self._max_count:
                        break
                count = min(self._max_count, count)
                robot_pose = self.obs_proxy.avg_robot_pose(
                    start_time, mid_end, roi=roi,
                    minute_increment=self.time_increment.secs/60
                )
                if not count:
                    full_obs = self.obs_proxy.is_robot_present_all_time(
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
                        "trajectory", int(self._max_count)
                    )
                    self._db.insert(
                        msg, {
                            "robot_pose": [
                                sum(zip(*robot_pose)[0]) / float(len(robot_pose)),
                                sum(zip(*robot_pose)[1]) / float(len(robot_pose))
                            ]
                        }
                    )
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_increment
        end_calc = rospy.Time.now()
        rospy.loginfo("Total calculation and storing time: %d" % (
            (end_calc - start_calc).secs
        ))


if __name__ == '__main__':
    rospy.init_node("trajectory_counter")
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
    src = TrajectoryCountObservation(
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
    src.store_trajectory_observation_per_time_increment(start_time, end_time)
