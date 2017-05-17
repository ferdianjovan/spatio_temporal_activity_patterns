#!/usr/bin/env python

import copy
import math
import rospy
import threading
import numpy as np
from scipy.fftpack import fft
from poisson_processes.rate import Lambda


_lock = threading.Lock()
_avg_acc_per_cycle = dict()


def get_xticks(start_time, periodic_cycle=10080):
    xticks = list()
    # Daily periodic
    if periodic_cycle == 1440:
        periodic_cycle *= 60
        start = (start_time.secs / periodic_cycle) * periodic_cycle
        step = start_time.secs
        counter = 0
        while start <= (
            ((start_time.secs / periodic_cycle) * periodic_cycle) + 2*periodic_cycle
        ):
            if start >= start_time.secs and start <= (start_time.secs + periodic_cycle):
                if counter < 10:
                    text = "0%d:00" % counter
                else:
                    text = "%d:00" % counter
                temp = ([""] * ((start - step) / 60)) + [text]
                xticks.extend(temp)
                step = start
            elif start > (start_time.secs + periodic_cycle):
                break
            start += 3600
            counter = (counter + 1) % 24
    return xticks


def rectify_wave(wave, up_thres=None, low_thres=None):
    for ind, val in enumerate(wave):
        if low_thres is not None and val < low_thres:
            wave[ind] = low_thres
        if up_thres is not None and val > up_thres:
            wave[ind] = up_thres
    return wave


def fourier_reconstruct(original, addition_method=True):
    num_of_freqs = min(len(original)/10, 30)
    if addition_method:
        spectrums, residue = _get_significant_frequencies(original, num_of_freqs*2)
        spectrums = spectrums[0:num_of_freqs]
    else:
        spectrums = _get_highest_n_freq(fft(original), num_of_freqs)
        residue = original
    reconstruction = 0
    for spectrum in spectrums:
        xf = np.linspace(0.0, len(original), len(original))  # frequency varations
        wave = spectrum[0] * np.cos((spectrum[2]*2.0*np.pi*xf) + spectrum[1])
        reconstruction += wave
    return reconstruction, residue


def _get_significant_frequencies(data, total_freq=15, max_addition=10, max_iteration=1000):
    N = len(data)
    xf = np.linspace(0.0, N, N)
    # initialise significant frequencies by taking frequency 0
    spectrum_data = fft(data)
    [amp, phs, freq] = _get_highest_n_freq(spectrum_data, 1)[0]
    frequencies = [[amp, phs, freq]]
    freq_occur_counter = {freq: 1}
    exit_counter = 0
    # data -= amp

    while len(frequencies) < total_freq:
        spectrum_data = fft(data)
        # recreate wave of the highest frequency
        [amp, phs, freq] = _get_highest_n_freq(spectrum_data, 2)[1]
        if freq == 0:
            [amp, phs, freq] = _get_highest_n_freq(spectrum_data, 2)[0]
        wave = amp * np.cos((freq * 2.0 * np.pi * xf) + phs)
        # substracting data with the wave
        data -= wave
        if freq not in zip(*frequencies)[2]:
            frequencies.append([amp, phs, freq])
            freq_occur_counter.update({freq: 1})
        else:
            for ind, val in enumerate(frequencies):
                if frequencies[ind][2] == freq and freq_occur_counter[freq] < max_addition:
                    frequencies[ind][0] += amp
                    frequencies[ind][1] = ((
                        freq_occur_counter[freq] * frequencies[ind][1]
                    ) + phs) / (freq_occur_counter[freq] + 1)
                    freq_occur_counter[freq] += 1
        exit_counter += 1
        if exit_counter >= max_iteration:
            break
    return frequencies, data


def _get_highest_n_freq(freqs, n=15):
    N = len(freqs)
    freqs = freqs[0:N/2]
    indices = [i for i in range(len(freqs))]
    angles = np.angle(freqs)
    amplitudes = np.abs(freqs) / float(N)
    sorted_result = sorted(zip(amplitudes, angles, indices), reverse=True)
    n_freqs = sorted_result[:n]
    return n_freqs


def find_n_nearest(array, value, n=3, start_index=0):
    if n > 0 and len(array) > 0:
        temp_value, idx = _find_nearest(array, value)
        left = find_n_nearest(array[:idx], value, n-1, start_index)
        left = [i for i in left if abs(i[1] - (start_index+idx)) > 3]
        right = find_n_nearest(array[idx+1:], value, n-1, start_index+idx+1)
        right = [i for i in right if abs(i[1] - (start_index+idx)) > 3]
        return left + [[temp_value, idx+start_index]] + right
    else:
        return list()


def _find_nearest(array, value):
    idx = (np.abs(array-value)).argmin()
    return array[idx], idx


def obtain_periodic_cycle(process, data, target_accuracy=0.8):
    global _avg_acc_per_cycle
    _avg_acc_per_cycle = dict()
    _obtain_periodic_cycle(process, data, target_accuracy)
    # print _avg_acc_per_cycle
    if len(_avg_acc_per_cycle.keys()) > 0:
        ordered_periodic_cycles = sorted(
            _avg_acc_per_cycle,
            key=lambda i: _avg_acc_per_cycle[i][0] * _avg_acc_per_cycle[i][1],
            reverse=True
        )
        recommended_period = ordered_periodic_cycles[0]
        avg_acc = (
            _avg_acc_per_cycle[recommended_period][0] * _avg_acc_per_cycle[recommended_period][1]
        )
        for i in ordered_periodic_cycles[1:]:
            acc = _avg_acc_per_cycle[i][0] * _avg_acc_per_cycle[i][1]
            cond = i < recommended_period
            cond = cond and abs(acc - avg_acc) <= target_accuracy/20.0
            if cond:
                recommended_period = i
        if avg_acc == 0.0:
            ordered_periodic_cycles = sorted(
                _avg_acc_per_cycle, key=lambda i: _avg_acc_per_cycle[i][1]
            )
            recommended_period = ordered_periodic_cycles[0]
    else:
        recommended_period = process.periodic_cycle
    return recommended_period


def _get_avg_acc(data, rates, lower, upper, target_accuracy):
    avg_acc = list()
    start_idx = 0
    accuracy_achieved_per_chunk = list()
    while start_idx < len(data):
        chunk = data[start_idx:start_idx+len(rates)]
        count = 0.0
        total = 0.0
        is_accuracy_achieved = False
        for idx, val in enumerate(chunk):
            if val <= math.ceil(rates[idx]+upper[idx]) and val >= math.floor(rates[idx]-lower[idx]):
                count += 1
            total += 1
        if float(count) / float(total) >= target_accuracy:
            is_accuracy_achieved = True
        avg_acc.append(float(count) / float(total))
        accuracy_achieved_per_chunk.append(is_accuracy_achieved)
        start_idx += len(rates)
    total_true = len([i for i in accuracy_achieved_per_chunk if i])
    avg_acc = sum(avg_acc) / float(len(avg_acc))
    return total_true / float(len(accuracy_achieved_per_chunk)), avg_acc


def _obtain_periodic_cycle(process, data, target_accuracy=0.8):
    global _avg_acc_per_cycle
    global _lock
    # set up _avg so other process does not do the same thing
    _lock.acquire()
    if process.periodic_cycle not in _avg_acc_per_cycle:
        rospy.loginfo("Periodic %d added" % process.periodic_cycle)
        _avg_acc_per_cycle[process.periodic_cycle] = (0.0, 0.0)
        _lock.release()
    else:
        _lock.release()
        return
    # setting up variables
    _prime = [2, 3, 5, 7, 11, 13]
    threads = {prime: None for prime in _prime}
    # data will be better if they start from rates[0]
    # rates, lower, upper = get_one_spectral_periodic_rate(process)
    rates, lower, upper = process.get_one_periodic_rate()
    if len(rates) < 1:
        rospy.loginfo("No stored process rate, returning process periodic")
        return
    data_values = [data[i] for i in sorted(data.keys())]
    acc, avg_acc = _get_avg_acc(data_values, rates, lower, upper, target_accuracy)
    rospy.loginfo(
        "Periodic %d produces %.3f accuracy with %.3f average" % (
            process.periodic_cycle, acc, avg_acc
        )
    )
    _lock.acquire()
    _avg_acc_per_cycle[process.periodic_cycle] = (acc, avg_acc)
    _lock.release()
    if acc < target_accuracy:
        for prime in _prime:
            if process.periodic_cycle * prime > process.max_periodic_cycle/2:
                rospy.loginfo(
                    "%d period is too long, ignore it." % (
                        process.periodic_cycle*prime
                    )
                )
                break
            copy_process = copy.deepcopy(process)
            # clear the original
            copy_process.poisson = dict()
            copy_process._init_time = None
            copy_process.periodic_cycle = process.periodic_cycle * prime
            # updating data
            for start_time in sorted(data.keys()):
                copy_process.update(start_time, data[start_time])
            threads[prime] = threading.Thread(
                target=_obtain_periodic_cycle,
                args=(copy_process, data, target_accuracy,)
            )
            threads[prime].start()
        for thread in threads.values():
            if thread is not None:
                thread.join()


def get_one_spectral_periodic_rate(process):
    rates, _ = fourier_reconstruct(process.get_one_periodic_rate()[0])
    start_time = process._init_time
    end_time = start_time + rospy.Duration(process.periodic_cycle*60)
    end_time = end_time + process.time_window - process.minute_increment
    scales = process.retrieve(start_time, end_time, scale=True)
    times = sorted(scales.keys())
    assert(len(times) == len(scales) == len(rates))
    copy_process = copy.deepcopy(process)
    copy_process.poisson = dict()
    for idx, rate in enumerate(rates):
        lmbda = Lambda()
        lmbda.reconstruct(rate, scales[times[idx]])
        copy_process.poisson[times[idx]] = lmbda
    return copy_process.get_one_periodic_rate()
