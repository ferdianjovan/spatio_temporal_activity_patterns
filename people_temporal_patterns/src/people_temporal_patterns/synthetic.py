#!/usr/bin/env python

import copy
import yaml
import time
import math
import rospy
import roslib
import random
import datetime
import threading
import numpy as np
from dateutil import parser
import matplotlib.pyplot as plt


class WeeklySyntheticData(object):

    def __init__(self, time_increment=1):
        config_path = roslib.packages.get_pkg_dir('people_temporal_patterns') + '/config/synthetic.yaml'
        self.weekly_shift = yaml.load(open(config_path, 'r'))
        # default
        self.map = "synthetic"
        self.config = "synthetic"
        rospy.loginfo("Map is %s with configuration %s" % (self.map, self.config))
        self.regions = {"1": list()}
        self.start_time = rospy.Time(
            time.mktime(datetime.datetime(2015, 5, 4, 0, 0).timetuple())
        )
        self.range_count = 5
        self.time_increment = time_increment
        self.current_count_per_region = {"1": dict()}

    def convert_weekly_shift(self, weekly_shift):
        result = list()
        for daily in weekly_shift:
            temp = list()
            for shift_time, count in daily.iteritems():
                st, et = shift_time.split("-")
                st = parser.parse(st)
                et = parser.parse(et)
                count += math.floor(random.gauss(0, 1))
                if count < 0:
                    count = 0
                temp.append((st, et, count))
            result.append(temp)
        return result

    def get_people_per_region(self, start_time, end_time):
        self.start_time = start_time
        result = {"1": dict()}
        if (end_time - start_time) >= rospy.Duration(604800):
            counter = (end_time - start_time).secs / 604800
            result["1"] = self.create_weekly_data(counter, True)
            rospy.loginfo("%d week-data are obtained" % counter)
        self.current_count_per_region = copy.deepcopy(result)
        thread = threading.Thread(target=self.plot)
        thread.start()
        return result

    def create_weekly_data(self, n_week=4, noise=False):
        result = dict()
        week_dur = 604800
        day_dur = 86400
        for week in range(n_week):
            weekly_shift = self.convert_weekly_shift(self.weekly_shift)
            for weekday in range(7):
                for minute in range(0, 1440, self.time_increment):
                    start_time = copy.copy(self.start_time)
                    start_time += (
                        rospy.Duration(week*week_dur) + rospy.Duration(weekday*day_dur)
                    )
                    start_time += rospy.Duration(minute*60)
                    start_date = datetime.datetime.fromtimestamp(start_time.secs)
                    count = 0
                    for [st, et, cnt] in weekly_shift[weekday]:
                        if st > et:
                            et = datetime.datetime(
                                start_date.year, start_date.month, start_date.day,
                                et.hour, et.minute
                            )
                            et = et + datetime.timedelta(hours=24)
                            et = datetime.datetime(et.year, et.month, et.day, 0, 0)
                        else:
                            et = datetime.datetime(
                                start_date.year, start_date.month, start_date.day,
                                et.hour, et.minute
                            )
                        st = datetime.datetime(
                            start_date.year, start_date.month, start_date.day,
                            st.hour, st.minute
                        )
                        if start_date >= st and start_date < et:
                            count = cnt
                            if noise and random.random() > 0.8:
                                count = cnt + math.ceil(random.gauss(0, 1))
                            if count < 0:
                                count = 0
                    result[start_time] = count
        return result

    def plot(self):
        if len(self.current_count_per_region["1"]) == 0:
            return
        data = self.current_count_per_region["1"]
        ordered_time = sorted(data.keys())
        y = list()
        for i in ordered_time:
            y.append(data[i])
        x = np.arange(len(y))
        plt.plot(
            x, y, "-", color="blue", label="Synthetic Data"
        )
        plt.title("4-week Synthetic Data")
        start_time = datetime.datetime.fromtimestamp(self.start_time.secs)
        plt.xlabel(
            """%d minute-period with %d minute increment
            Starting at %s""" % (len(y), self.time_increment, str(start_time))
        )
        plt.ylabel("Count")
        plt.ylim(ymin=-1)
        plt.legend()
        plt.show()
