#!/usr/bin/env python

import argparse

import yaml
import rospy
import roslib
from scipy.stats import nbinom, binom
from detection_observation.detection_count import DetectionCountObservation


class MultiDetectionProbability(object):

    # window represents the time interval for Poisson distribution.
    # increment represent the sub time interval.
    def __init__(
        self, config, increment=rospy.Duration(60),
        window=rospy.Duration(600), max_count_per_increment=1
    ):
        if window < increment:
            rospy.logwarn("Time window can't be smaller than the time increment")
            rospy.logwarn("Set window to 10 minutes, and interval to 1 minute...")
            window = rospy.Duration(600)
            increment = rospy.Duration(60)
        self.time_window = window
        self.time_increment = increment
        self.total_bins = (window.secs * max_count_per_increment) / increment.secs
        self.drc = DetectionCountObservation(
            config, self.time_increment, max_count_per_increment
        )
        self.detect_likelihood = self.read_detection_likelihood_from_file()
        rospy.loginfo(
            "TPR and TNR for each detection: %s" % str(self.detect_likelihood)
        )

    def read_detection_likelihood_from_file(self):
        detectors = yaml.load(
            open(
                roslib.packages.get_pkg_dir(
                    'multi_detect_temporal_patterns'
                ) + '/config/detectors.yaml',
                'r'
            )
        )["detectors"]
        detect_likelihood = dict()
        for detection_type, detection in detectors.iteritems():
            # TP = P(detection=1 | event=1)
            tp = detection["tp"]
            # TN = P(detection=0 | event=0)
            tn = detection["tn"]
            detect_likelihood[detection_type] = {
                (1, 1): tp, (0, 0): tn,
                (0, 1): (1 - tp), (1, 0): (1 - tn)
            }
        return detect_likelihood

    def get_detection_likelihood_for_single_observation(
        self, observation, occurrence, detection_type
    ):
        sums = 0
        # P(obs | occ, total_bins) = Sigma_{sa=0}^{obs} binom(sa | occ, tp) * binom((obs-sa) | (total_bins-occ), fp)
        for sa in range(occurrence+1):
            tp_fn_prob = binom.pmf(
                sa, occurrence, self.detect_likelihood[detection_type][1, 1]
            )
            fp_tn_prob = binom.pmf(
                (observation-sa), (self.total_bins-occurrence),
                self.detect_likelihood[detection_type][1, 0]
            )
            sums += (tp_fn_prob * fp_tn_prob)
        return sums

    def get_single_occurrence_likelihood(
        self, observations, occurrence, alpha, beta
    ):
        # max occurrence value can not exceed total_bins
        occurrence = min(occurrence, self.total_bins)
        occ_likelihood = dict()
        for i in range(self.total_bins+1):
            # P(x_i)
            prod = nbinom.pmf(i, alpha, beta / float(beta + 1))
            # product of P(s_{ij} | x_i)
            for dtype, obs in observations.iteritems():
                sensor_likelihood = self.get_detection_likelihood_for_single_observation(
                    obs, i, dtype
                )
                prod *= sensor_likelihood
            occ_likelihood[i] = prod
        occ_likelihood = occ_likelihood[occurrence] / float(
            sum(occ_likelihood.values())
        )
        return occ_likelihood

    def get_all_occurrence_likelihood_within_time(
        self, start_time, end_time, alpha=1.1, beta=1, roi=""
    ):
        all_occ = list()
        observations = dict()
        for detector in self.detect_likelihood:
            obs = self.drc.load_observation(
                start_time, end_time, roi, detector
            )
            observations[detector] = sum([o.count for o in obs])
        rospy.loginfo("Observations: %s" % observations)
        for occ in range(self.total_bins+1):
            occ_likelihood = self.get_single_occurrence_likelihood(
                observations, occ, alpha, beta
            )
            all_occ.append((occ, occ_likelihood))
        rospy.loginfo("Probability occurrence: %s" % str(all_occ))
        return all_occ, observations


if __name__ == '__main__':
    rospy.init_node("multiple_detection_count")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    parser.add_argument(
        "-w", dest="time_window", default="10",
        help="Interval time (in minute). Default is 10 minutes"
    )
    args = parser.parse_args()
    mco = MultiDetectionProbability(
        args.soma_config, rospy.Duration(int(args.time_increment)*60),
        rospy.Duration(int(args.time_window)*60)
    )
    sums = 0
    # check P(s | x)
    for i in range(mco.total_bins+1):
        temp = mco.get_detection_likelihood_for_single_observation(i, 3, "scene")
        sums += temp
        print i, temp
    # check P(x | (s_1, ..., s_m))
    # observations = {"scene": 3}
    # for i in range(mco.total_bins+1):
    #     temp = mco.get_single_occurrence_likelihood(observations, i, 1.1, 1)
    #     sums += temp
    #     print i, temp
    # check get_all_occurrence_likelihood_within_time
    # start_time = rospy.Time(
    #     time.mktime(
    #         datetime.datetime(2017, 1, 1, 0, 0).timetuple()
    #     )
    # )
    # end_time = rospy.Time(
    #     time.mktime(
    #         datetime.datetime(2017, 1, 1, 0, 10).timetuple()
    #     )
    # )
    # result = mco.get_all_occurrence_likelihood_within_time(
    #     start_time, end_time, alpha=1.1, beta=1, roi="region_simulation"
    # )
    # for i in result:
    #     print i
    #     sums += i[1]
    print "sum: ", sums
