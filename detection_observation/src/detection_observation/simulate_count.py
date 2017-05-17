#!/usr/bin/env python

import yaml
import time
import rospy
import roslib
import datetime
import argparse
import numpy as np
from mongodb_store.message_store import MessageStoreProxy
from detection_observation.msg import DetectionObservation
from detection_observation.detection_count import DetectionCountObservation


class SimulateCountObservation(object):

    def __init__(
        self, rate=1, increment=rospy.Duration(60),
        window=rospy.Duration(600), max_count_per_increment=1
    ):
        rospy.loginfo("Initiating Counting Simulation...")
        self.map = "simulation"
        self.config = "simulation"
        self.time_increment = increment
        self.time_window = window
        self._max_count = max_count_per_increment
        self._rate = rate
        self._db = MessageStoreProxy(collection="detection_observation")
        self.det_prob = self.calculate_detection_prob()

    def load_observation(self, start_time, end_time, detection_type=""):
        start_time = start_time.secs / 60
        start_time = rospy.Time(start_time * 60)
        end_time = end_time.secs / 60
        end_time = rospy.Time(end_time * 60)
        meta = {"rate": self._rate}
        query = {
            "soma": self.map, "soma_config": self.config, "max_value": self._max_count,
            "start_time.secs": {"$gte": start_time.secs, "$lt": end_time.secs},
            "region_id": "region_simulation"
        }
        if detection_type != "":
            query.update({"detection_type": detection_type})
        logs = self._db.query(
            DetectionObservation._type, message_query=query,
            meta_query=meta, sort_query=[("start_time.secs", 1)]
        )
        return [log[0] for log in logs]

    def calculate_detection_prob(self):
        likelihood = yaml.load(
            open(
                roslib.packages.get_pkg_dir(
                    'detection_observation'
                ) + '/config/simulate.yaml',
                'r'
            )
        )
        rospy.loginfo("Detector's Accuracy: %s" % str(likelihood["detectors"]))
        return likelihood["detectors"]

    def store_simulate_observation_per_time_window(self, start_time, end_time):
        rospy.loginfo(
            "Storing simulate observation from %s to %s" % (
                str(datetime.datetime.fromtimestamp(start_time.secs)),
                str(datetime.datetime.fromtimestamp(end_time.secs))
            )
        )
        mid_end = start_time + self.time_window
        activity_counter = [0, 0]  # (activity, no_activity)
        carry_over = 0
        while mid_end <= end_time:
            count = np.random.poisson(self._rate) + carry_over
            carry_over = 0
            if count > self._max_count*(self.time_window.secs/60):
                carry_over = count - self._max_count*(self.time_window.secs/60)
                rospy.loginfo("OVERLOAD: %d, CARRY OVER: %d" % (count, carry_over))
                count -= carry_over
            activity_counter[0] += count
            activity_counter[1] += (((self.time_window.secs/60) * self._max_count) - count)
            carry_over += self.store_simulate_observation_per_time_increment(
                start_time, mid_end, count
            )
            start_time = start_time + self.time_window
            mid_end = start_time + self.time_window
        simulate = {
            "activity": activity_counter[0] / float(sum(activity_counter)),
            "detectors": self.det_prob
        }
        with open(
            roslib.packages.get_pkg_dir(
                'detection_observation'
            ) + '/config/simulate.yaml', 'w'
        ) as f:
            f.write(yaml.dump(simulate))

    def store_simulate_observation_per_time_increment(self, start_time, end_time, count):
        mid_end = start_time + self.time_increment
        while mid_end <= end_time:
            temp = count / float(
                ((end_time-start_time).secs)/60*self._max_count
            )
            if temp >= 0.51:
                curr_count = self._max_count
            else:
                curr_count = sum([
                    1 for i in range(self._max_count) if np.random.random() <= 0.5
                ])
                curr_count = min(count, curr_count)
            count -= curr_count
            self._store_simulate_observation(start_time, mid_end, curr_count)
            start_time = start_time + self.time_increment
            mid_end = start_time + self.time_increment
        return count

    def _store_simulate_observation(self, start_time, end_time, count):
        detector_count = dict()
        for i in range(count):
            for detector, prob in self.det_prob.iteritems():
                if detector not in detector_count:
                    detector_count[detector] = 0
                rand = np.random.random()
                if rand <= prob["tp"]:
                    detector_count[detector] += 1
        for i in range(self._max_count - count):
            for detector, prob in self.det_prob.iteritems():
                if detector not in detector_count:
                    detector_count[detector] = 0
                rand = np.random.random()
                if rand > prob["tn"]:
                    detector_count[detector] += 1
        for det_name, det_count in detector_count.iteritems():
            msg = DetectionObservation(
                self.map, self.config, "region_simulation", start_time,
                (end_time-rospy.Duration(0, 1)), det_count,
                det_name, int(self._max_count)
            )
            self._db.insert(msg, {"rate": self._rate})


if __name__ == '__main__':
    rospy.init_node("simulate_counter")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument(
        "-r", dest="rate", default="1",
        help="Arrival rate per time increment. Default is 1."
    )
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute."
    )
    parser.add_argument(
        "-c", dest="max_count", default="1",
        help="Maximum counting value per time increment. Default is 1."
    )
    args = parser.parse_args()
    src = SimulateCountObservation(
        rate=int(args.rate), increment=rospy.Duration(int(args.time_increment)*60),
        max_count_per_increment=int(args.max_count)
    )
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
    src.store_simulate_observation_per_time_window(start_time, end_time)
