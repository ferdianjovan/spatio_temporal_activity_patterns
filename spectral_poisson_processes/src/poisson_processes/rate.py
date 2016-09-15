#!/usr/bin/env python

from scipy.stats import gamma


class Lambda(object):

    def __init__(self, interval=1, confidence_rate=0.95):
        self.reset()
        self.interval = interval
        self.confidence_rate = confidence_rate

    def reset(self):
        self.scale = 1.0
        self.shape = 1.1
        self._gamma_map = self._gamma_mode(self.shape, self.scale)
        self._gamma_mean = gamma.mean(self.shape, scale=1/float(self.scale))

    def update_lambda(self, data):
        self.shape += sum(data)
        self.scale += len(data) * self.interval
        self._gamma_map = self._gamma_mode(self.shape, self.scale)
        self._gamma_mean = gamma.mean(self.shape, scale=1/float(self.scale))

    def _gamma_mode(self, shape, scale):
        if shape >= 1:
            return (shape - 1) / float(scale)
        else:
            return -1.0

    def get_rate(self):
        return self._gamma_map

    def upper_end(self):
        return gamma.ppf(self.confidence_rate, self.shape, scale=1/float(self.scale))

    def lower_end(self):
        return gamma.ppf((1-self.confidence_rate), self.shape, scale=1/float(self.scale))

    def set_rate(self, value):
        self._gamma_map = value

    def reconstruct(self, value, scale):
        if value >= 0:
            self.scale = scale
            self._gamma_map = value
            self.shape = (value * scale) + 1
