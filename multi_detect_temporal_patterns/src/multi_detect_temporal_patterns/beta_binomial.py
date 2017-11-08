#! /usr/bin/env python

from scipy.special import betaln
from scipy.misc import comb
import pymongo
import roslib
import rospy
import yaml
import math


class BetaBinomial():

    # modelling based on 'simulatiion' or 'real_data'
    # 'tpr' for true positive probability and 'fpr' for false positive probability
    def __init__(self, data_source, no_sub_intervals):
        sensors = yaml.load(
            open(
                roslib.packages.get_pkg_dir(
                    'multi_detect_temporal_patterns'
                ) + '/config/sensors.yaml',
                'r'
            )
        ).split(",")
        self.sensors = [i.replace(" ", "") for i in sensors]
        path = roslib.packages.get_pkg_dir('multi_detect_temporal_patterns')
        self.sub_intervals = no_sub_intervals
        if data_source == 'real_data':
            # path to activity file for ground truth
            with open(path + '/data/positive_count.yaml', 'r') as f:
                self.true_events = yaml.load(f)
            # path to non-activity file for ground truth
            with open(path + '/data/negative_count.yaml', 'r') as f:
                self.no_events = yaml.load(f)
            # path to way points
            with open(path + '/data/wp_history.yaml', 'r') as f:
                self.wp_history = yaml.load(f)

            req_prob = ['tpr', 'fpr']
            self.alphaD = dict()
            self.betaD = dict()

            for sensor in self.sensors:
                prob_dict_a = dict()
                prob_dict_b = dict()
                for prob in req_prob:
                    alpha_dict, beta_dict = self.calc_alpha_beta_realdata(sensor, prob)
                    prob_dict_a.update({prob: alpha_dict})
                    prob_dict_b.update({prob: beta_dict})

                self.alphaD.update({sensor: prob_dict_a})
                self.betaD.update({sensor: prob_dict_b})
        # For testing
        elif data_source == 'simulation':
            # fix mongo details to point to simulation source
            client = pymongo.MongoClient('localhost', 62345)
            rospy.loginfo('Finding simulation data...')
            self.data = client.message_store.detection_observation.find()
            req_prob = ['tpr', 'fpr']
            self.alphaD = dict()
            self.betaD = dict()
            for sensor in self.sensors:
                prob_dict_a = dict()
                prob_dict_b = dict()
                for prob in req_prob:
                    alpha, beta = self.calc_alpha_beta_simdata(sensor, prob)
                    prob_dict_a.update({prob: alpha})
                    prob_dict_b.update({prob: beta})
                    self.data.rewind()
                self.alphaD.update({sensor: prob_dict_a})
                self.betaD.update({sensor: prob_dict_b})

    def _update_dict(self, dictionary, region, event):
        if region not in dictionary:
            dictionary.update({region: dict()})

        waypoint_dict = dictionary[region]
        if self.wp_history[event] not in waypoint_dict:
            val = 1
        else:
            val = waypoint_dict[self.wp_history[event]] + 1

        waypoint_dict.update({self.wp_history[event]: val})
        dictionary.update({region: waypoint_dict})

        return dictionary

    # sensor should be specified as ubd, cd, leg
    def calc_alpha_beta_realdata(self, sensor, req_prob):
        alpha_dict = dict()
        beta_dict = dict()

        path = roslib.packages.get_pkg_dir('multi_detect_temporal_patterns')
        with open(path + '/data/'+sensor+'.yaml', 'r') as f:
            sensor_detections = yaml.load(f)

        for region in sensor_detections:
            detections = sensor_detections[region]
            if req_prob == 'tpr':
                for event in self.true_events[region]:
                    if event in detections:
                        alpha_dict = self._update_dict(alpha_dict, region, event)
                    else:
                        beta_dict = self._update_dict(beta_dict, region, event)
            if req_prob == 'fpr':
                for event in self.no_events[region]:
                    if event in detections:
                        alpha_dict = self._update_dict(alpha_dict, region, event)
                    else:
                        beta_dict = self._update_dict(beta_dict, region, event)
        return alpha_dict, beta_dict

    # sensor should be specified as scene, scene2 -> simulation has only one region and
    # one waypoint for observation.
    def calc_alpha_beta_simdata(self, sensor, req_prob):
        alpha = 1
        beta = 1
        for observation in self.data:
            if observation['detection_type'] == sensor:
                if req_prob == 'tpr':
                    if observation['count'] == 1 and observation['_meta']['real_count'] == 1:
                        alpha = alpha + 1
                    elif observation['count'] == 0 and observation['_meta']['real_count'] == 1:
                        beta = beta + 1
                elif req_prob == 'fpr':
                    if observation['count'] == 1 and observation['_meta']['real_count'] == 0:
                        alpha = alpha+1
                    elif observation['count'] == 0 and observation['_meta']['real_count'] == 0:
                        beta = beta + 1
        return float(alpha), float(beta)

    # Beta Binomial function {P(x) = bin(tp)+bin(fp)}
    # where n is the total no.of trials
    # alpha is successes
    # beta is failures
    # i is the value of the count variable x
    def betaBinomial(self, i, n, alpha, beta):
        j = comb(n, i, 1) * math.exp((betaln(i+alpha, n-i+beta) - betaln(alpha, beta)))
        return j

    def get_possible_waypoint(self, region):
        result = list()
        for sensor in self.sensors:
            for req_prob in ['tpr', 'fpr']:
                if region in self.alphaD[sensor][req_prob]:
                    result.extend(self.alphaD[sensor][req_prob][region].keys())
                if region in self.betaD[sensor][req_prob]:
                    result.extend(self.betaD[sensor][req_prob][region].keys())
        return list(set(result))

    # Probability of sji given xi. xi -> count of real events
    # sji -> sensor detections  = tp +fp
    # tp can take values from 0 to xi
    # fp = sji - tp can take values from sji-xi to sji
    # l is the number of sub-intervals
    def prob_s_x(self, sji, xi, sensor, region=None, waypoint=None):
        alpha = []
        beta = []
        if region is not None and waypoint is not None:
            for req_prob in ['tpr', 'fpr']:
                if region not in self.alphaD[sensor][req_prob]:
                    alpha.append(1)
                elif waypoint not in self.alphaD[sensor][req_prob][region]:
                    alpha.append(1)
                else:
                    alpha.append(self.alphaD[sensor][req_prob][region][waypoint])

                if region not in self.betaD[sensor][req_prob]:
                    beta.append(1)
                elif waypoint not in self.betaD[sensor][req_prob][region]:
                    beta.append(1)
                else:
                    beta.append(self.betaD[sensor][req_prob][region][waypoint])
            prob = self.doubleBinomial(sji, xi, alpha, beta)
        else:
            for req_prob in ['tpr', 'fpr']:
                alpha.append(self.alphaD[sensor][req_prob])
                beta.append(self.betaD[sensor][req_prob])
            prob = self.doubleBinomial(sji, xi, alpha, beta)
        return prob

    def doubleBinomial(self, sji, xi, alpha, beta):
        prob = 0.0
        if sji >= xi:
            limit = xi
        else:
            limit = sji

        for i in range(limit+1):
            betaBinom_tp = self.betaBinomial(i, xi, alpha[0], beta[0])
            betaBinom_fp = self.betaBinomial(sji-i, self.sub_intervals-xi, alpha[1], beta[1])
            if betaBinom_fp == 0 or betaBinom_tp == 0:
                prob = prob + 0
            else:
                prob = prob + math.exp((math.log(betaBinom_tp)+math.log(betaBinom_fp)))
        return prob


if __name__ == '__main__':
    # simulation or real_data, no. of sub intervals
    # bb1 = BetaBinomial('real_data', 10)
    # # sji, xi, sensor (str), region (str), waypoint (str)
    # prob = bb1.prob_s_x(0, 2, 'upper_body', '2', 'ChargingPoint1')
    # print 'Real:', prob
    # simulation or real_data, no. of sub intervals
    bb2 = BetaBinomial('simulation', 10)
    # sji, xi, sensor (str), region (str), waypoint (str)
    prob = bb2.prob_s_x(0, 2, 'scene')
    print 'Sim scene:', prob
    prob = bb2.prob_s_x(0, 2, 'scene2')
    print 'Sim scene2:', prob
