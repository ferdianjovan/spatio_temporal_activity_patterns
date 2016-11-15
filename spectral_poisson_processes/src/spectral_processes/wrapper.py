#!/usr/bin/env python

from poisson_processes.wrapper import PoissonWrapper
from spectral_processes.processes import SpectralPoissonProcesses


class SpectralPoissonWrapper(PoissonWrapper):

    def __init__(
        self, topic, topic_msg, topic_attribute, value, not_value=False,
        meta_info=dict(), max_count=None, window=10, increment=1,
        periodic_cycle=10080
    ):
        super(SpectralPoissonWrapper, self).__init__(
            topic, topic_msg, topic_attribute, value, not_value, meta_info,
            max_count, window, increment, periodic_cycle
        )
        # changing self.process should be fine as long as self.time_window is
        # not zero
        self.process = SpectralPoissonProcesses(window, increment, periodic_cycle)
