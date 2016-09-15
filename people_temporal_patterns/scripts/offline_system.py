#!/usr/bin/env python

import time
import rospy
import argparse
import datetime
from poisson_processes.processes import PeriodicPoissonProcesses
from spectral_processes.processes import SpectralPoissonProcesses
from people_temporal_patterns.synthetic import WeeklySyntheticData
from people_temporal_patterns.offline_counter import PeopleCounter


class OfflinePeopleCounter(object):

    def __init__(
        self, config, window, increment, periodic_cycle,
        with_fourier=False, synthetic_data=False
    ):
        self.with_fourier = with_fourier
        self.synthetic = synthetic_data
        self.offline = PeopleCounter(
            config, window, increment, periodic_cycle, with_fourier
        )
        if synthetic_data:
            rospy.loginfo("Using synthetic data, reconfigure soma information")
            wsd = WeeklySyntheticData(increment)
            self.offline.region_people_counter = wsd
            if with_fourier:
                self.offline.process = {
                    roi: SpectralPoissonProcesses(
                        # window, increment, periodic_cycle, periodic_cycle
                        window, increment, periodic_cycle
                    ) for roi in wsd.regions.keys()
                }
            else:
                self.offline.process = {
                    roi: PeriodicPoissonProcesses(
                        window, increment, periodic_cycle
                    ) for roi in wsd.regions.keys()
                }

    def execute(self, mode):
        isloaded = self.offline.load_from_db()
        if mode:
            region = raw_input("Region %s: " % str(self.offline.process.keys()))
            self.offline.plot_poisson_per_region(region, self.with_fourier)
            # self.offline.plot_scale_per_region(region)
        else:
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
            if not isloaded:
                self.offline.construct_process_from_trajectory(
                    start_time, end_time
                )
            if self.with_fourier:
                try_auto = raw_input("Try auto cycle finding [yes(1), no(0)]:")
                if int(try_auto):
                    for region in self.offline.process.keys():
                        try_auto = raw_input("Try auto cycle on region %s? yes(1) or no(0)" % region)
                        if int(try_auto):
                            self.offline.reconstruct_periodic_cycle_per_region(
                                region, start_time, end_time
                            )
            self.offline.store_to_db()


if __name__ == '__main__':
    rospy.init_node("offline_people_poisson")
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
    parser.add_argument(
        "-p", dest="plot", default="0",
        help="Choose to plot figure(1) or construct(0). Default is construct(0)"
    )
    parser.add_argument(
        "-s", dest="synthetic", default="0",
        help="Choose to use synthetic data(1) or real data(0). Default is real data(0)"
    )
    args = parser.parse_args()

    system = OfflinePeopleCounter(
        args.soma_config, int(args.time_window),
        int(args.time_increment), int(args.periodic_cycle), int(args.model),
        int(args.synthetic)
    )
    system.execute(int(args.plot))
