#!/usr/bin/env python

import copy
import rospy
import argparse
import numpy as np
import matplotlib.pyplot as plt
from region_observation.util import get_soma_info
from poisson_processes.processes import PeriodicPoissonProcesses
from spectral_processes.processes import SpectralPoissonProcesses
from spectral_processes.util import fourier_reconstruct, rectify_wave, get_xticks


class ScenePlot(object):

    def __init__(self, config, window=10, increment=1, periodic_cycle=10080, with_fourier=False):
        rospy.loginfo("Starting the offline scene temporal model reconstruction...")
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
                "region_id": roi, "type": "scene"
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
        reconstruction, residue = fourier_reconstruct(original)
        reconstruction = rectify_wave(reconstruction, low_thres=0.01)
        return start_time, reconstruction, original, residue

    def plot_poisson_per_region(self, region, with_fourier=True):
        if with_fourier:
            name = "Poisson-Spectral Process"
            start_time, y, _, up_err = self.get_rate_rate_err_per_region(
                region, False
            )
            if len(y) == 0:
                return
            x = np.arange(len(y))
            # y = map(lambda i: i*4.5, y)
            line = plt.plot(x, y, "-", color="b", label=name)
            plt.setp(line, linewidth=3)
            up_err = np.array(up_err) + np.array(y)
            line = plt.plot(x, up_err, "-", color="r", label="Upper Bound")
            plt.setp(line, linewidth=3)
        else:
            name = "Poisson Process"
            start_time, y, _, up_err = self.get_rate_rate_err_per_region(
                region, True
            )
            if len(y) == 0:
                return
            x = np.arange(len(y))
            # y = map(lambda i: i*4.5, y)
            line = plt.plot(x, y, "-", color="b", label=name)
            plt.setp(line, linewidth=3)
            up_err = np.array(up_err) + np.array(y)
            line = plt.plot(x, up_err, "-", color="r", label="Upper Bound")
            plt.setp(line, linewidth=3)

        plt.title(
            "%s for Region %s, Scene Changes" % (name, region),
            # "%s for Region 17, Upper Bodies" % (name),
            # "%s for Region 17, Scene Changes" % (name),
            fontsize=30
        )
        plt.xlabel(
            "Periodic time with %d minute time window" % (
                self.process[region].time_window.secs/60
            ), fontsize=25
        )
        plt.xticks(
            x, get_xticks(start_time, self.periodic_cycle),
            rotation="horizontal", fontsize=15
        )
        plt.ylabel("Arrival Rate", fontsize=25)
        plt.yticks(fontsize=15)
        plt.ylim(ymin=-1)
        plt.legend(prop={'size': 25}, loc='best')
        plt.show()


if __name__ == '__main__':
    rospy.init_node("scene_temporal_plot")
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
    system = ScenePlot(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle), int(args.model)
    )
    region = raw_input("Region %s: " % str(system.process.keys()))
    system.plot_poisson_per_region(region, int(args.model))
