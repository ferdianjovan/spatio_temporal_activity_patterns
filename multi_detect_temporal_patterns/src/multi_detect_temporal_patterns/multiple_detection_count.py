#!/usr/bin/env python

import time
import datetime
import argparse

import yaml
import rospy
import roslib
from truths import Truths
from detection_observation.msg import DetectionObservation
from detection_observation.detection_count import DetectionCountObservation


# http://stackoverflow.com/questions/15580291/how-to-efficiently-calculate-a-row-in-pascals-triangle
def pascal(n):
    line = [1]
    for k in range(n):
        line.append(line[k] * (n-k) / (k+1))
    return line


class MultipleDetectionCountObservation(object):

    def __init__(self, config, increment=rospy.Duration(60)):
        self.time_increment = increment
        self.detection_types = list()
        self.drc = DetectionCountObservation(config, self.time_increment, 1)
        self.likelihood = yaml.load(
            open(
                roslib.packages.get_pkg_dir(
                    'stationary_temporal_patterns'
                ) + '/config/detectors.yaml',
                'r'
            )
        )
        self.act_prob = self.calculate_detection_prob()

    def calculate_detection_prob(self):
        result = dict()
        activity = self.likelihood["activity"]
        detection_probs = list()
        for detection_type, detection in self.likelihood["detectors"].iteritems():
            self.detection_types.append(detection_type)
            detection_probs.append(detection)
        combinations = Truths(self.detection_types)
        for combi in combinations.base_conditions:
            numerator = activity
            denominator = (1 - activity)
            key = ""
            for det_ind, base in enumerate(combi):
                if base:
                    numerator *= detection_probs[det_ind]["tp"]
                    denominator *= (1-detection_probs[det_ind]["tn"])
                    key += "-%s" % self.detection_types[det_ind]
                else:
                    numerator *= (1-detection_probs[det_ind]["tp"])
                    denominator *= detection_probs[det_ind]["tn"]
                    key += "-n%s" % self.detection_types[det_ind]
            denominator += numerator
            result[key[1:]] = numerator / float(denominator)
        return result

    def calculate_detection_value(self, detections):
        key = ""
        for i, det_type in enumerate(self.detection_types):
            if detections[i][0].count:
                key += "-%s" % det_type
            else:
                key += "-n%s" % det_type
        return self.act_prob[key[1:]]

    def get_marginal_probability_count_within(self, start_time, end_time, roi=""):
        result = list([(0, 1)])
        detections = list()
        detection_lengths = list()
        detection_padding_ind = list()
        zero_detections = list()
        # initiation
        for detector in self.detection_types:
            observations = self.drc.load_observation(
                start_time, end_time, roi, detector
            )
            detections.append(observations)
            detection_lengths.append(len(observations))
            detection_padding_ind.append(0)
            zero_detections.append(0)
        # calculation
        for ind in range(max(detection_lengths)):
            temp_detection = [(DetectionObservation(), i) for i, _ in enumerate(self.detection_types)]
            for det_ind in range(len(self.detection_types)):
                if ind < detection_lengths[det_ind]:
                    detection = detections[det_ind][
                        ind+detection_padding_ind[det_ind]
                    ]
                    temp_detection[det_ind] = (detection, det_ind)
                if not temp_detection[det_ind][0].count:
                    zero_detections[det_ind] += 1
            temp_detection = sorted(temp_detection, key=lambda i: i[0].start_time.secs)
            time_detection = 0
            for i, (detection, det_ind) in enumerate(temp_detection):
                if detection.start_time.secs == 0:
                    continue
                elif not time_detection:
                    time_detection = detection.start_time.secs
                elif detection.start_time.secs > time_detection:
                    temp_detection[i] = (DetectionObservation(), det_ind)
                    detection_padding_ind[det_ind] = -1
                else:
                    assert detection.start_time.secs >= time_detection, "The previous sorting does not work properly."
                    time_detection = detection.start_time.secs
            temp_detection = sorted(temp_detection, key=lambda i: i[1])
            p = self.calculate_detection_value(temp_detection)
            temp = list()
            for i in result:
                temp.append((i[0]+1, i[1] * p))
                temp.append((i[0], i[1] * (1-p)))
            result = temp
        # no detection and robot did not stay the entire time, dont count
        cond = True
        for ind, zero_detection in enumerate(zero_detections):
            cond = cond and (zero_detection == detection_lengths[ind])
        cond = cond and max(detection_lengths) < (end_time-start_time).secs/60
        if cond:
            result = list([(0, 1)])
        return result


if __name__ == '__main__':
    rospy.init_node("multiple_detection_count")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument("soma_config", help="Soma configuration")
    parser.add_argument(
        "-i", dest="time_increment", default="1",
        help="Incremental time (in minute). Default is 1 minute"
    )
    args = parser.parse_args()
    mco = MultipleDetectionCountObservation(
        "poisson_activity", rospy.Duration(int(args.time_increment)*60)
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
    roi = raw_input("ROI %s: " % mco.drc.regions.keys())
    print mco.get_marginal_probability_count_within(start_time, end_time, roi)
