#!/usr/bin/env python

import random
import datetime
import numpy as np
from scipy.stats import gamma, entropy

import rospy
from region_observation.util import get_soma_info
from spectral_processes.processes import SpectralPoissonProcesses
from multi_detect_temporal_patterns.independent_count_posterior import CountPosterior


class MultiDetectionCounter(object):

    def __init__(
        self, config, window=10, increment=1, periodic_cycle=10080
    ):
        rospy.loginfo("Starting multi-sensor counter...")
        if window < increment:
            rospy.logwarn("Time window can't be smaller than the time increment")
            rospy.logwarn("Set window to 10 minutes, and interval to 1 minute...")
            window = 10
            increment = 1
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
        self.mdc = CountPosterior(
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
        self.load_from_db()
        rospy.Timer(self.time_increment, self.update)

    def update(self, event):
        end_time = rospy.Time(
            ((rospy.Time.now().secs / 60) * 60) - 60
        )
        start_time = end_time - self.time_window
        self.updating_temporal_patterns(start_time)

    def gamma_approximation(self, x, sums, lmbd, max_epoch=100):
        max_x = x[np.where(sums == max(sums))]
        beta = lmbd.scale + 1.0
        alpha = (max_x * beta) + 1.0
        epoch = 0
        err = 1000
        prev_err = 1000
        lbeta = beta - (beta * 0.25)
        ubeta = beta + (beta * 0.25)
        while epoch < max_epoch:
            temp_beta = random.uniform(lbeta, ubeta)
            temp_alpha = (max_x * temp_beta) + 1.0
            y = gamma.pdf(x, temp_alpha, scale=1/temp_beta)
            err = entropy(sums, y, base=2)
            if err < prev_err:
                epoch = 0
                prev_err = err
                beta = temp_beta
                alpha = temp_alpha
                lbeta = beta - (beta * 0.25)
                ubeta = beta + (beta * 0.25)
            else:
                epoch += 1
        return alpha, beta, prev_err

    def updating_temporal_patterns(self, start_time)
        mid_end = start_time + self.time_window
        rospy.loginfo(
            "Updating multi-detection processes for each region from %s to %s" % (
                datetime.datetime.fromtimestamp(start_time.secs),
                datetime.datetime.fromtimestamp(mid_end.secs)
            )
        )
        stopwatch_start = rospy.Time.now()
        for roi in self.regions:
            lmbd = self.process[roi].get_lambda_at(start_time)
            x = np.linspace(
                gamma.ppf(0.00001, lmbd.shape, scale=1/lmbd.scale),
                gamma.ppf(0.99, lmbd.shape, scale=1/lmbd.scale),  1000
            )
            sums = 0
            beta = lmbd.scale + 1.0
            detections, obs = self.mdc.count_posterior_probability(
                start_time, mid_end, roi=roi, alpha=lmbd.shape, beta=lmbd.scale
            )
            rospy.loginfo(
                "Region %s has %s at %s" % (
                    roi, obs, datetime.datetime.fromtimestamp(start_time.secs)
                )
            )
            rospy.loginfo("Count posterior probability: %s" % str(detections))
            if len(detections):
                for detection in detections:
                    alpha = lmbd.shape + detection[0]
                    sums += gamma.pdf(x, alpha, scale=1/beta) * detection[1]
                alpha, beta, _ = self.gamma_approximation(x, sums, lmbd)
                shape = alpha
                scale = beta
                lmbd.shape = 0.0
                lmbd.scale = scale - 1.0
                self.process[roi].set_lambda_at(start_time, lmbd)
                self.process[roi].update(start_time, shape)
                self._store(roi, start_time)
        stopwatch_stop = rospy.Time.now()
        rospy.loginfo("Elapsed Time: %d seconds, %d nanoseconds" % (
            (stopwatch_stop - stopwatch_start).secs, (stopwatch_stop - stopwatch_start).nsecs
        ))

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
        rospy.loginfo("Retrieving multi-sensor counter from database. It may take a while...")
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
    rospy.init_node("multi_detection_counter")
    sensor_counter = DetectionCounter(
        rospy.get_param("~soma_config", "poisson_activity"),
        rospy.get_param("~time_window", 10),
        rospy.get_param("~time_increment", 1),
        rospy.get_param("~periodic_cycle", 1440)
    )
    rospy.spin()
