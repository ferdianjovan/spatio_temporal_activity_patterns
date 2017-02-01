#!/usr/bin/env python

import copy
import rospy
import argparse
import datetime
import numpy as np
import matplotlib.pyplot as plt
from region_observation.util import get_soma_info
from poisson_processes.processes import PeriodicPoissonProcesses
from spectral_processes.processes import SpectralPoissonProcesses
from spectral_processes.util import fourier_reconstruct, rectify_wave


class MultidetectionPlot(object):

    def __init__(self, config, window=10, increment=1, periodic_cycle=10080, with_fourier=False):
        rospy.loginfo("Starting the offline multi-detection temporal model reconstruction...")
        self._start_time = None
        self.time_window = window
        self.time_increment = increment
        self.periodic_cycle = periodic_cycle
        self.soma_config = config
        self.regions, self.map = get_soma_info(config)
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        if with_fourier:
            self.process = {
                roi: SpectralPoissonProcesses(
                    window, increment, periodic_cycle
                ) for roi in self.regions.keys()
            }
        else:
            self.process = {
                roi: PeriodicPoissonProcesses(
                    window, increment, periodic_cycle
                ) for roi in self.regions.keys()
            }
        self.load_from_db()

    def load_from_db(self):
        rospy.loginfo("Retrieving from database...")
        is_retrieved = list()
        for roi in self.process.keys():
            meta = {
                "soma_map": self.map,
                "soma_config": self.soma_config,
                "region_id": roi, "type": "multi-detection"
            }
            is_retrieved.append(self.process[roi].retrieve_from_mongo(meta))
        for roi, process in self.process.iteritems():
            cond = process._init_time is not None
            cond = cond and (
                self._start_time is None or self._start_time > process._init_time
            )
            if cond:
                self._start_time = process._init_time
        if True in is_retrieved:
            rospy.loginfo("Starting time is %s" % self._start_time.secs)
        return (True in is_retrieved)

    def get_rate_rate_err_per_region(self, region, with_poisson=False):
        if with_poisson:
            process = PeriodicPoissonProcesses(
                self.time_window, self.time_increment, self.periodic_cycle
            )
            process.poisson = copy.deepcopy(self.process[region].poisson)
            process._init_time = self.process[region]._init_time
            process._prev_init = self.process[region]._prev_init
            result = process.get_one_periodic_rate()
        else:
            result = self.process[region].get_one_periodic_rate()
        rates, lower_bounds, upper_bounds = result
        start_time = self.process[region]._init_time
        return start_time, rates, lower_bounds, upper_bounds

    def fourier_reconstruction(self, region):
        start_time, original, _, _ = self.get_rate_rate_err_per_region(region)
        # num of frequency chosen is 1/10 of the length of data
        reconstruction, residue = fourier_reconstruct(original)
        reconstruction = rectify_wave(reconstruction, low_thres=0.01)
        return start_time, reconstruction, original, residue

    def plot_poisson_per_region(self, region, with_fourier=True):
        start_time, y, low_err, up_err = self.get_rate_rate_err_per_region(region)
        if len(y) == 0:
            return
        x = np.arange(len(y))
        line = plt.plot(x, y, "-", color="b", label="Poisson Process")
        plt.setp(line, linewidth=2)
        up_err = np.array(up_err) + np.array(y)
        line = plt.plot(x, up_err, "-", color="c", label="Upper Bound")
        plt.setp(line, linewidth=2)
        if with_fourier:
            start_time, y, _, up_err = self.get_rate_rate_err_per_region(
                region, True
            )
            line = plt.plot(x, y, "-", color="r", label="Without Fourier Poisson Process")
            plt.setp(line, linewidth=2)
            up_err = np.array(up_err) + np.array(y)
            line = plt.plot(x, up_err, "-", color="m", label="Without Fourier Upper Bound")
            plt.setp(line, linewidth=2)
        else:
            _, reconstruction, _, _ = self.fourier_reconstruction(region)
            line = plt.plot(
                x, reconstruction, "-", color="green", label="Fourier Reconstruction"
            )
            plt.setp(line, linewidth=2)
        plt.title("Multi Detection Poisson Process for Region %s" % region)
        start_time = datetime.datetime.fromtimestamp(start_time.secs)
        plt.xlabel(
            """%d minute-period with %d minute increment and %d minute time window
            Starting at %s""" % (
                self.process[region].periodic_cycle,
                self.process[region].minute_increment.secs/60,
                self.process[region].time_window.secs/60,
                str(start_time)
            )
        )
        plt.ylabel("Poisson Rate")
        plt.ylim(ymin=-1)
        plt.legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node("multi_detect_temporal_plot")
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
    parser.add_argument(
        "-m", dest="model", default="0",
        help="Model to use, Poisson (0) or Spectral Poisson(1). Default is Poisson(0)"
    )
    args = parser.parse_args()
    system = MultidetectionPlot(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle), int(args.model)
    )
    region = raw_input("Region %s: " % str(system.process.keys()))
    system.plot_poisson_per_region(region, int(args.model))
