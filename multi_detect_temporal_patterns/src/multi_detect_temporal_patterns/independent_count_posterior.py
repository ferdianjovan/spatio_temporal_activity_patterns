#!/usr/bin/env python

import time
import datetime
import argparse
from scipy.stats import nbinom
from shapely.geometry import Polygon, Point

import rospy
from strands_navigation_msgs.msg import TopologicalMap
from multi_detect_temporal_patterns.beta_binomial import BetaBinomial
from detection_observation.detection_count import DetectionCountObservation


class CountPosterior(object):

    # window represents the time interval for Poisson distribution.
    # increment represent the sub time interval.
    def __init__(
        self, config, increment=rospy.Duration(60),
        window=rospy.Duration(600), max_count_per_increment=1,
        data="real_data"
    ):
        # Topological Map WayPoint stuff for sensor model (you can have sensor
        # model without being specific to waypoint)
        self.topo_map = None
        self._get_waypoints()
        self.topo_map = {
            wp.name: Polygon(
                [[wp.pose.position.x+i.x, wp.pose.position.y+i.y] for i in wp.verts]
            ) for wp in self.topo_map.nodes
        }
        # real stuff (does not work without this)
        self.time_window = window
        self.time_increment = increment
        self.total_bins = (window.secs * max_count_per_increment) / increment.secs
        self.drc = DetectionCountObservation(
            config, self.time_increment, max_count_per_increment
        )
        rospy.loginfo("Calculating a sensor model for each region. It might take a while...")
        self.sensor_model = BetaBinomial(data, self.total_bins)

    def _topo_map_cb(self, topo_map):
        self.topo_map = topo_map

    def _get_waypoints(self):
        topo_sub = rospy.Subscriber(
            "/topological_map", TopologicalMap, self._topo_map_cb, None, 10
        )
        rospy.loginfo("Getting information from /topological_map...")
        while self.topo_map is None:
            rospy.sleep(0.1)
        topo_sub.unregister()

    def find_closest_waypoint(self, robot_pose, roi):
        possible_wps = self.sensor_model.get_possible_waypoint(roi)
        wps = [
            (
                wp, robot_pose.distance(area)
            ) for wp, area in self.topo_map.iteritems() if wp in possible_wps
        ]
        wps = sorted(wps[:5], key=lambda i: i[1])
        return wps[0][0]

    def count_posterior_probability(
        self, start_time, end_time, alpha=1.1, beta=1, roi=""
    ):
        posterior = list()
        observations = dict()
        waypoint = ""
        for detector in self.sensor_model.sensors:
            obs = self.drc.load_observation(
                start_time, end_time, roi, detector, True
            )
            observations[detector] = sum([o[0].count for o in obs])
            num_of_obs = len(obs)
            if num_of_obs:
                robot_pose = Point([
                    sum([o[1]["robot_pose"][0] for o in obs]) / num_of_obs,
                    sum([o[1]["robot_pose"][1] for o in obs]) / num_of_obs
                ])
                waypoint = self.find_closest_waypoint(robot_pose, roi)
        if waypoint != "":
            sums = 0.0
            for occ in range(self.total_bins+1):
                prod = nbinom.pmf(occ, alpha, beta / float(beta + 1))
                for dtype, obs in observations.iteritems():
                    prod *= self.sensor_model.prob_s_x(
                        obs, occ, dtype, roi, waypoint
                    )
                sums += prod
                posterior.append((occ, prod))
            posterior = [(i[0], i[1] / float(sums)) for i in posterior]
        return posterior, observations

    def count_posterior_probability_simulation(
        self, start_time, end_time, alpha=1.1, beta=1, roi="region_simulation"
    ):
        posterior = list()
        observations = dict()
        for detector in self.sensor_model.sensors:
            obs = self.drc.load_observation(
                start_time, end_time, roi, detector, True
            )
            observations[detector] = sum([o[0].count for o in obs])
        sums = 0.0
        for occ in range(self.total_bins+1):
            prod = nbinom.pmf(occ, alpha, beta / float(beta + 1))
            for dtype, obs in observations.iteritems():
                prod *= self.sensor_model.prob_s_x(
                    obs, occ, dtype
                )
            sums += prod
            posterior.append((occ, prod))
        posterior = [(i[0], i[1] / float(sums)) for i in posterior]
        rospy.loginfo("Count posterior probability: %s" % str(posterior))
        return posterior, observations


if __name__ == '__main__':
    rospy.init_node("independent_posterior_count")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Interval time (in minute). Default is 10 minutes"
    )
    args = parser.parse_args()
    mco = CountPosterior(
        "simulation", rospy.Duration(int(args.time_increment)*60),
        rospy.Duration(int(args.time_window)*60), 1, "simulation"
    )
    start_time = rospy.Time(
        time.mktime(
            datetime.datetime(2017, 1, 1, 0, 40).timetuple()
        )
    )
    end_time = rospy.Time(
        time.mktime(
            datetime.datetime(2017, 1, 1, 0, 50).timetuple()
        )
    )
    stopwatch_start = rospy.Time.now()
    result = mco.count_posterior_probability_simulation(
        start_time, end_time, alpha=1.1, beta=1
    )
    stopwatch_stop = rospy.Time.now()
    rospy.loginfo("Elapsed Time: %d seconds, %d nanoseconds" % (
        (stopwatch_stop - stopwatch_start).secs, (stopwatch_stop - stopwatch_start).nsecs
    ))
    sums = 0.0
    for i in result[0]:
        sums += i[1]
    print "sums: %.2f" % sums
