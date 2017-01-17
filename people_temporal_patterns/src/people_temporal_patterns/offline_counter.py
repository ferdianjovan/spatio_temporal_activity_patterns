#!/usr/bin/env python

import copy
import rospy
import datetime
import numpy as np
import matplotlib.pyplot as plt
from poisson_processes.processes import PeriodicPoissonProcesses
from spectral_processes.processes import SpectralPoissonProcesses
from people_temporal_patterns.people_count import RegionPeopleCount
from spectral_processes.util import fourier_reconstruct, rectify_wave


class PeopleCounter(object):

    def __init__(self, config, window=10, increment=1, periodic_cycle=10080, with_fourier=False):
        rospy.loginfo("Starting the offline poisson reconstruction...")
        self._start_time = None
        self.time_window = window
        self.time_increment = increment
        self.periodic_cycle = periodic_cycle
        # get soma-related info
        self.region_people_counter = RegionPeopleCount(
            config, window, increment
        )
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        if with_fourier:
            self.process = {
                roi: SpectralPoissonProcesses(
                    window, increment, periodic_cycle
                ) for roi in self.region_people_counter.regions.keys()
            }
        else:
            self.process = {
                roi: PeriodicPoissonProcesses(
                    window, increment, periodic_cycle
                ) for roi in self.region_people_counter.regions.keys()
            }

    def store_to_db(self, process_ids=list()):
        if process_ids == list():
            process_ids = self.process.keys()
        for roi in process_ids:
            rospy.loginfo("Storing process %s to database..." % roi)
            meta = {
                "soma_map": self.region_people_counter.map,
                "soma_config": self.region_people_counter.config,
                "region_id": roi, "type": "people"
            }
            self.process[roi].store_to_mongo(meta)

    def load_from_db(self):
        rospy.loginfo("Retrieving from database...")
        is_retrieved = list()
        for roi in self.process.keys():
            meta = {
                "soma_map": self.region_people_counter.map,
                "soma_config": self.region_people_counter.config,
                "region_id": roi, "type": "people"
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

    def reconstruct_periodic_cycle_per_region(
        self, region, start_time=None, end_time=None
    ):
        data = self.region_people_counter.current_count_per_region[region]
        if len(data.keys()) == 0 and start_time is not None and end_time is not None:
            data = self.region_people_counter.get_people_per_region(
                start_time, end_time
            )
            data = self.region_people_counter.current_count_per_region[region]
        rospy.loginfo("Reconstructing periodic cycle for region %s" % region)
        self.process[region].reconstruct_periodic_cycle(data)

    def construct_process_from_trajectory(self, start_time, end_time, regions=list()):
        count_per_region = self.region_people_counter.get_people_per_region(
            start_time, end_time
        )
        if regions != list():
            count_per_region = {
                roi: val for roi, val in count_per_region.iteritems() if roi in regions
            }
        rospy.loginfo("Updating the poisson processes for each region")
        for roi, time_count in count_per_region.iteritems():
            ordered = sorted(time_count.keys())
            for start_time in ordered:
                self.process[roi].update(start_time, time_count[start_time])
        for roi, process in self.process.iteritems():
            if regions == list() or roi in regions:
                cond = process._init_time is not None
                cond = cond and (
                    self._start_time is None or self._start_time > process._init_time
                )
                if cond:
                    self._start_time = process._init_time

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

    def get_scale_per_region(self, region):
        start_time = self._start_time
        end_time = start_time + rospy.Duration(
            self.process[region].periodic_cycle*60
        )
        scales = self.process[region].retrieve(
            start_time, end_time, use_upper_confidence=False, scale=True
        )
        scale_plan = list()
        for key in sorted(scales.keys()):
            scale_plan.append(scales[key])
        return start_time, scale_plan

    def plot_scale_per_region(self, region):
        start_time, scale = self.get_scale_per_region(region)
        if len(scale) == 0:
            return
        x = np.arange(len(scale))
        plt.plot(
            x, scale, "-x", color="green", label="Visiting Plan"
        )
        plt.title("Total number of visits for Region %s" % region)
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
        plt.ylabel("The number of visits")
        plt.ylim(ymin=-1)
        plt.legend()
        plt.show()

    def fourier_reconstruction(self, region):
        start_time, original, _, _ = self.get_rate_rate_err_per_region(region)
        # num of frequency chosen is 1/10 of the length of data
        reconstruction, residue = fourier_reconstruct(original)
        reconstruction = rectify_wave(reconstruction,low_thres=0.01)
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
        # y = map(lambda i: i*2.5, y)
        # plt.errorbar(
        #     x, y, yerr=[low_err, up_err], color='b', ecolor='r',
        #     fmt="-o", label="Poisson Process"
        # )
        plt.title("People Poisson Process for Region %s" % region)
        # plt.title("Poisson Process of the Corridor", fontsize=40)
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
        # plt.xlabel("One Week Period", fontsize=40)
        # xticks = ([""] * 469) + ["Tuesday"] + ([""] * 970)
        # xticks += ([""] * 469) + ["Wednesday"] + ([""] * 970)
        # xticks += ([""] * 469) + ["Thursday"] + ([""] * 970)
        # xticks += ([""] * 469) + ["Friday"] + ([""] * 970)
        # xticks += ([""] * 469) + ["Saturday"] + ([""] * 970)
        # xticks += ([""] * 469) + ["Sunday"] + ([""] * 970)
        # xticks += ([""] * 469) + ["Monday"] + ([""] * 970)
        # plt.xticks(x, xticks, rotation="horizontal", fontsize=30)
        # plt.yticks(fontsize=30)
        # plt.xticks([])
        plt.ylabel("Poisson Rate")
        # plt.ylabel("Poisson Rate", fontsize=40)
        # plt.yticks([])
        plt.ylim(ymin=-1)
        plt.legend()
        # plt.legend(prop={'size': 40})
        plt.show()
