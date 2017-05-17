#!/usr/bin/env python

# import yaml
import time
import rospy
import random
# import pickle
# import getpass
import argparse
import datetime
import numpy as np
from scipy.stats import gamma
from region_observation.util import get_soma_info
from spectral_processes.processes import SpectralPoissonProcesses
from multi_detect_temporal_patterns.multiple_detection_probability import MultiDetectionProbability


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
        self.mdc = MultiDetectionProbability(
            self.config, self.time_increment, self.time_window, 1
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
        # lmbd_evolve = {"shape": list(), "scale": list(), "rate": list()}
        # temp_start_time = start_time
        mid_end = start_time + self.time_window
        # sums_gamma_list = list()
        # x_sums_gamma_list = list()
        # approx_gamma_list = list()
        # x_approx_gamma_list = list()
        while start_time < end_time:
            for roi in self.regions:
                lmbd = self.process[roi].get_lambda_at(start_time)
                detections, observations = self.mdc.get_all_occurrence_likelihood_within_time(
                    start_time, mid_end, roi=roi, alpha=lmbd.shape, beta=lmbd.scale
                )
                # different way to calculate total shape and scale
                # brute force finding approximate gamma distribution
                acceptable_err = 0.005  # for PEAK
                # acceptable_err = 0.01  # for AREA
                prev_err = 1000
                err = 1000
                sums = 0
                x = np.linspace(
                    gamma.ppf(0.01, lmbd.shape, scale=1/lmbd.scale) / 2,
                    gamma.ppf(0.99, lmbd.shape, scale=1/lmbd.scale) * 2,  100
                )
                beta = lmbd.scale + 1.0
                for detection in detections:
                    alpha = lmbd.shape + detection[0]
                    sums += gamma.pdf(x, alpha, scale=1/beta) * detection[1]
                max_x = x[np.where(sums == max(sums))]
                max_y = max(sums)
                # area = np.trapz(sums, dx=1)
                alpha = (max_x * beta) + 1.0
                lbeta = beta - (beta * 0.25)
                ubeta = beta + (beta * 0.25)
                while err > acceptable_err:
                    beta = random.uniform(lbeta, ubeta)
                    alpha = (max_x * beta) + 1.0
                    y = gamma.pdf(x, alpha, scale=1/beta)
                    nmax_y = max(y)
                    # narea = np.trapz(y, dx=1)
                    prev_err = err
                    err = abs(max_y - nmax_y)
                    # err = abs(area - narea)
                    if err < prev_err:
                        lbeta = beta - (beta * 0.25)
                        ubeta = beta + (beta * 0.25)
                rospy.loginfo(
                    "mode: %.2f, mode's probability: %.2f, approximate's probability: %.2f, err: %.2f" % (
                        max_x, max_y, nmax_y, err
                    )
                )
                # rospy.loginfo(
                #     "mode: %.2f, sum's area: %.2f, gamma approximate area: %.2f, err: %.2f" % (
                #         max_x, area, narea, err
                #     )
                # )
                shape = alpha
                scale = beta
                lmbd.shape = 0.0
                lmbd.scale = scale - 1.0
                self.process[roi].set_lambda_at(start_time, lmbd)
                self.process[roi].update(start_time, shape)
                self._store(roi, start_time)
                # if (start_time-temp_start_time).secs % self.time_window.secs == 0:
                #     lmbd = self.process[roi].get_lambda_at(start_time)
                #     lmbd_evolve["shape"].append(float(lmbd.shape))
                #     lmbd_evolve["scale"].append(float(lmbd.scale))
                #     lmbd_evolve["rate"].append(float(lmbd.get_rate()))
                #     sums_gamma_list.append(sums)
                #     x_sums_gamma_list.append(x)
                #     approx_gamma_list.append(y)
                #     x_approx_gamma_list.append(x)
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_window
        # with open("/home/%s/Pictures/multi-detect.yaml" % getpass.getuser(), 'w') as f:
        #     f.write(yaml.dump(lmbd_evolve))
        # with open(
        #     "/home/%s/Pictures/multi-detect_sums-evolution.p" % getpass.getuser(), 'w'
        # ) as f:
        #     pickle.dump(
        #         {
        #             "sums": sums_gamma_list, "approx": approx_gamma_list,
        #             "x_sums": x_sums_gamma_list, "x_approx": x_approx_gamma_list
        #         },
        #         f, pickle.HIGHEST_PROTOCOL
        #     )

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
