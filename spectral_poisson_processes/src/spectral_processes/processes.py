#!/usr/bin/env python

import copy
import rospy
import numpy as np
from poisson_processes.rate import Lambda
from poisson_processes.processes import PeriodicPoissonProcesses
from spectral_processes.util import obtain_periodic_cycle, fourier_reconstruct
from spectral_processes.util import _get_significant_frequencies, find_n_nearest, rectify_wave


class SpectralPoissonProcesses(PeriodicPoissonProcesses):

    def __init__(
        self, time_window=10, minute_increment=1, periodic_cycle=10080,
        max_periodic_cycle=40320, coll="spectral_processes"
    ):
        self._is_updated = False
        self._spectral_process = None
        self._update_count = 0
        self._init_cycle = periodic_cycle
        self._prime_numbers = [2, 3, 5, 7, 11]
        self.max_periodic_cycle = max_periodic_cycle
        super(SpectralPoissonProcesses, self).__init__(
            time_window, minute_increment, self._init_cycle, coll
        )

    def retrieve_from_mongo(self, meta=dict()):
        count = 2
        is_retrieved = super(SpectralPoissonProcesses, self).retrieve_from_mongo(meta)
        while not is_retrieved and self.periodic_cycle <= self.max_periodic_cycle:
            self.periodic_cycle = self._init_cycle * count
            is_retrieved = super(
                SpectralPoissonProcesses, self
            ).retrieve_from_mongo(meta)
            count += 1
        if not is_retrieved:
            self.periodic_cycle = self._init_cycle
        if self._init_time is not None:
            self._fourier_reconstruct()
        return is_retrieved

    def update(self, start_time, count):
        super(SpectralPoissonProcesses, self).update(start_time, count)
        delta_now = (start_time - self._init_time)
        delta_periodic = self.minute_increment * (self._update_count * self.periodic_cycle)
        self._is_updated = True
        if delta_now >= delta_periodic:
            self._update_count += 1
            self._fourier_reconstruct()

    def _fourier_reconstruct(self):
        rospy.loginfo("Spectral process reconstruction...")
        start_time = self._init_time
        end_time = start_time + rospy.Duration(self.periodic_cycle*60)
        end_time = end_time + self.time_window - self.minute_increment
        poisson = super(SpectralPoissonProcesses, self).retrieve(start_time, end_time)
        keys = sorted(poisson.keys())
        rates = list()
        for key in keys:
            rates.append(poisson[key])
        rates, _ = fourier_reconstruct(rates)
        rates = rectify_wave(rates, low_thres=Lambda().get_rate())
        scales = super(SpectralPoissonProcesses, self).retrieve(
            start_time, end_time, scale=True
        )
        times = sorted(scales.keys())
        self._spectral_process = PeriodicPoissonProcesses(
            (self.time_window.secs)/60, (self.minute_increment.secs)/60,
            self.periodic_cycle
        )
        self._spectral_process._init_time = self._init_time
        self._spectral_process._prev_init = self._prev_init
        self._spectral_process.poisson = dict()
        for idx, rate in enumerate(rates):
            if rate > Lambda().get_rate():
                lmbda = Lambda()
                lmbda.reconstruct(rate, scales[times[idx]])
                self._spectral_process.poisson[times[idx]] = lmbda
            else:
                self._spectral_process.poisson[times[idx]] = Lambda()
        self._is_updated = False

    def retrieve(
        self, start_time, end_time, use_upper_confidence=False,
        use_lower_confidence=False, scale=False
    ):
        if self._init_time is not None:
            if self._is_updated:
                self._fourier_reconstruct()
            return self._spectral_process.retrieve(
                start_time, end_time, use_upper_confidence,
                use_lower_confidence, scale
            )
        return dict()

    def reconstruct_periodic_cycle(self, data):
        # data = {start_time: count} ideally is as long as the original
        # self.poisson (covers all periodic time)
        # keep the copy
        _prev_periodic_cycle = copy.copy(self.periodic_cycle)
        # recommended_periodic = self.obtain_periodic_cycle(data)  # return recommended_periodic, accuracy based on data
        recommended_periodic = obtain_periodic_cycle(self, data, target_accuracy=0.80)
        rospy.loginfo(
            "Initial cycle: %s, Recommended cycle: %s" % (
                _prev_periodic_cycle, recommended_periodic
            )
        )
        if _prev_periodic_cycle != recommended_periodic:
            self.periodic_cycle = recommended_periodic
        else:
            self.periodic_cycle = _prev_periodic_cycle
        self.poisson = dict()
        self._init_time = None
        # updating data
        ordered_time = sorted(data.keys())
        for start_time in ordered_time:
            self.update(start_time, data[start_time])

    def is_freqs_inside_folds(self, data, freqs, fold=2):
        result = True
        temp_data = copy.copy(data)
        spectrum, _ = _get_significant_frequencies(temp_data)
        spectrum = [i for i in spectrum if i[2] == fold]
        if len(spectrum) > 0:
            spectrum = spectrum[0]
        else:
            rospy.logwarn("The fold %d is not part of the highest spectrum in the data" % fold)
            return not result
        xf = np.linspace(0.0, len(data), len(data))
        wave = spectrum[0] * np.cos((spectrum[2]*2.0*np.pi*xf) + spectrum[1])
        idxs = sorted(zip(*find_n_nearest(wave, spectrum[0], fold+1))[1])
        if len(idxs) <= 1:
            rospy.logwarn("Data can not be folded")
            return not result
        for i in range(1, len(idxs)):
            temp_data = copy.copy(data)
            spectrums, _ = _get_significant_frequencies(temp_data[idxs[i-1]:idxs[i]])
            spectrums = zip(*spectrums)[2]
            for freq in freqs:
                if freq not in spectrums:
                    result = False
                    break
            if not result:
                break
        return result

    def obtain_periodic_cycle(self, data):
        total_freq = 6
        periodic_cycles = list()
        _prev_periodic_cycle = copy.copy(self.periodic_cycle)
        while _prev_periodic_cycle < self.max_periodic_cycle:
            ori_rates, _, _ = self.get_one_periodic_rate()
            ori_spectrums, _ = _get_significant_frequencies(ori_rates, total_freq=total_freq)
            for prime in self._prime_numbers:
                if _prev_periodic_cycle * prime > self.max_periodic_cycle/2:
                    break
                # clear the original
                self.poisson = dict()
                self._init_time = None
                self.periodic_cycle = _prev_periodic_cycle * prime
                # updating data
                ordered_time = sorted(data.keys())
                for start_time in ordered_time:
                    self.update(start_time, data[start_time])
                rates, _, _ = self.get_one_periodic_rate()
                if not self.is_freqs_inside_folds(
                    rates, zip(*ori_spectrums)[2], fold=prime
                ):
                    periodic_cycles.append(self.periodic_cycle)
            if len(periodic_cycles) > 0:
                rospy.loginfo(
                    "Periodic cycles: %s" % str(sorted(periodic_cycles))
                )
                # _prev_periodic_cycle = max(periodic_cycles)
                _prev_periodic_cycle = min(periodic_cycles)
                self.poisson = dict()
                self._init_time = None
                self.periodic_cycle = _prev_periodic_cycle
                # updating data
                ordered_time = sorted(data.keys())
                for start_time in ordered_time:
                    self.update(start_time, data[start_time])
            else:
                break
            periodic_cycles = list()
        return _prev_periodic_cycle
