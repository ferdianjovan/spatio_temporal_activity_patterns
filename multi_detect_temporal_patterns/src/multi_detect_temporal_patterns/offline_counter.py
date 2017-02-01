#!/usr/bin/env python

import time
import rospy
import argparse
import datetime
from region_observation.util import get_soma_info
from spectral_processes.processes import SpectralPoissonProcesses
from stationary_temporal_patterns.multiple_detection_count import MultipleDetectionCountObservation


class MultiDetectionCounter(object):

    def __init__(
        self, config, window=10, increment=1, periodic_cycle=10080
    ):
        rospy.loginfo("Starting multi-detection processes...")
        self._start_time = None
        # get soma-related info
        self.config = config
        self.regions, self.map = get_soma_info(config)
        # for each roi create PoissonProcesses
        rospy.loginfo(
            "Time window is %d minute with increment %d minute" % (
                window, increment
            )
        )
        self.periodic_cycle = periodic_cycle
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        self.mdc = MultipleDetectionCountObservation(
            self.config, self.time_increment
        )
        rospy.loginfo(
            "Creating a periodic cycle every %d minutes" % periodic_cycle
        )
        self.process = {
            roi: SpectralPoissonProcesses(
                window, increment, periodic_cycle
            ) for roi in self.regions.keys()
        }

    def learn_mdp_pattern(self, start_time, end_time):
        rospy.loginfo(
            "Updating multi-detection processes for each region from %s to %s" % (
                datetime.datetime.fromtimestamp(start_time.secs),
                datetime.datetime.fromtimestamp(end_time.secs)
            )
        )
        mid_end = start_time + self.time_window
        while start_time < end_time:
            for roi in self.regions:
                detections = self.mdc.get_marginal_probability_count_within(
                    start_time, mid_end, roi
                )
                if len(detections) > 1:
                    lmbd = self.process[roi].get_lambda_at(start_time)
                    numerator = 0
                    denominator = 0
                    # Welch-Satterthwaite approx applied bayesian update
                    for detection in detections:
                        numerator += (lmbd.shape+detection[0]) / (
                            (lmbd.scale+1.0)/float(detection[1])
                        )
                        denominator += (lmbd.shape+detection[0]) / (
                            (lmbd.scale+1.0)/float(detection[1])
                        )**2
                    shape = numerator**2 / denominator
                    scale = numerator / denominator
                    # hacking way to trick the Lambda.update
                    # assuming the interval is full time window.
                    lmbd.shape = 0.0
                    lmbd.scale = scale - 1.0
                    self.process[roi].set_lambda_at(start_time, lmbd)
                    self.process[roi].update(start_time, shape)
                    self._store(roi, start_time)
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_window

    def _store(self, roi, start_time):
        self.process[roi]._store(
            start_time,
            {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "multi-detection"
            }
        )

    def retrieve_from_to(
        self, start_time, end_time, use_upper_confidence=False, scale=False
    ):
        result = dict()
        for roi, poisson in self.process.iteritems():
            result.update(
                # use upper confidence rate value
                {roi: poisson.retrieve(
                    start_time, end_time,
                    use_upper_confidence=use_upper_confidence, scale=scale
                )}
            )
        return result

    def load_from_db(self):
        rospy.loginfo("Retrieving people processes from database. It may take a while...")
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "multi-detection"
            }
            self.process[roi].retrieve_from_mongo(meta)

    def store_to_db(self):
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map, "soma_config": self.config,
                "region_id": roi, "type": "multi-detection"
            }
            self.process[roi].store_to_mongo(meta)


if __name__ == '__main__':
    rospy.init_node("offline_multi_detection_counter")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Fixed time window interval (in minute) for each Poisson distribution. Default is 10 minutes"
    )
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    parser.add_argument(
        "-c", dest="periodic_cycle", default="10080",
        help="Desired periodic cycle (in minute). Default is one week (10080 minutes)"
    )
    args = parser.parse_args()
    sc = MultiDetectionCounter(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle)
    )
    sc.load_from_db()
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
    sc.learn_mdp_pattern(start_time, end_time)
