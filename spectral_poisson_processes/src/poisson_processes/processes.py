#!/usr/bin/env python


import time
import copy
import rospy
import datetime
from poisson_processes.rate import Lambda
from spectral_poisson_processes.msg import ProcessMsg
from mongodb_store.message_store import MessageStoreProxy


class PoissonProcesses(object):

    def __init__(self, time_window=10, minute_increment=1, coll="poisson_processes"):
        # time_window and minute_increment are in minutes
        try:
            if 60 % time_window != 0 and 60 % minute_increment != 0:
                rospy.logwarn("Time window and minute increment are not factors of 60")
                rospy.logwarn("Using default ones (time window = 10 minutes, increment = 1 minute)")
                time_window = 10
                minute_increment = 1
        except:
            rospy.logwarn("Time window and minute increment can not be zero")
            rospy.logwarn("Using default ones (time window = 10 minutes, increment = 1 minute)")
            time_window = 10
            minute_increment = 1

        self._init_time = None
        self._prev_init = None
        self.poisson = dict()
        self.time_window = rospy.Duration(time_window*60)
        self.minute_increment = rospy.Duration(minute_increment*60)
        self._db = MessageStoreProxy(collection=coll)

    def get_lambda_at(self, start_time):
        start_time = self._convert_time(start_time)
        end_time = start_time + self.time_window
        key = "%s-%s" % (start_time.secs, end_time.secs)
        if key not in self.poisson:
            lmbd = Lambda()
        else:
            lmbd = self.poisson[key]
        return lmbd

    def set_lambda_at(self, start_time, lmbd):
        start_time = self._convert_time(start_time)
        end_time = start_time + self.time_window
        key = "%s-%s" % (start_time.secs, end_time.secs)
        self.poisson[key] = lmbd

    def default_lambda(self):
        return Lambda()

    def update(self, start_time, count):
        # no end time, assuming that the count was taken within time window
        # starting from the init_time
        start_time = self._convert_time(start_time)
        end_time = start_time + self.time_window
        if self._init_time is None or start_time < self._init_time:
            self._prev_init = copy.copy(self._init_time)
            self._init_time = start_time
        key = "%s-%s" % (start_time.secs, end_time.secs)
        if key not in self.poisson:
            self.poisson[key] = Lambda()
        self.poisson[key].update_lambda([count])

    def retrieve(
        self, start_time, end_time, use_upper_confidence=False,
        use_lower_confidence=False, scale=False
    ):
        """ retrieve poisson distribution from specified start_time until specified end_time
            minus time window interval.
        """
        # convert start_time and end_time to the closest time range (in minutes)
        start_time = self._convert_time(start_time)
        end_time = self._convert_time(end_time)
        # rospy.loginfo(
        #     "Retrieving Poisson model from %d to %d" % (start_time.secs, end_time.secs)
        # )
        if self._init_time is None:
            rospy.logwarn("Retrieving data is not possible, no poisson model has been learnt.")
            return
        end_time = end_time - self.time_window
        result = dict()
        while start_time <= end_time:
            mid_end = start_time + self.time_window
            key = "%s-%s" % (start_time.secs, mid_end.secs)
            try:
                poisson = self.poisson[key]
            except:
                poisson = self.default_lambda()
            if not scale:
                # upper trumphs lower
                if use_upper_confidence:
                    result[key] = poisson.upper_end()
                elif use_lower_confidence:
                    result[key] = poisson.lower_end()
                else:
                    result[key] = poisson.get_rate()
            else:
                result[key] = poisson.scale
            start_time = start_time + self.minute_increment
        return result

    def _convert_time(self, start_time):
        return rospy.Time((start_time.secs/60)*60)

    def store_to_mongo(self, meta=dict()):
        rospy.loginfo("Storing all poisson data...")
        for key in self.poisson.iterkeys():
            start_time = rospy.Time(int(key.split("-")[0]))
            self._store(start_time, meta)

    def _store(self, start_time, meta):
        if self._init_time is None:
            rospy.logwarn("Storing data is not possible, no poisson model has been learnt.")
            return
        start_time = self._convert_time(start_time)
        start = datetime.datetime.fromtimestamp(start_time.secs)
        end_time = start_time + self.time_window
        key = "%s-%s" % (start_time.secs, end_time.secs)
        lmbd = self.poisson[key]
        msg = ProcessMsg(
            start.month, start.day, start.hour, start.minute,
            self.time_window, lmbd.shape, lmbd.scale, lmbd.get_rate()
        )

        meta.update({"start": self._init_time.secs, "year": start.year})
        # print "Storing %s with meta %s" % (str(msg), str(meta))
        query = {
            "month": start.month, "day": start.day, "hour": start.hour,
            "minute": start.minute, "duration.secs": self.time_window.secs
        }
        meta_query = copy.copy(meta)
        if self._prev_init is not None:
            meta_query["start"] = self._prev_init.secs
            total_retrieved = len(
                self._db.query(ProcessMsg._type, query, meta_query)
            )
            if total_retrieved < 1:
                meta_query["start"] = self._init_time.secs
        self._db.update(msg, meta, query, meta_query, True)

    def retrieve_from_mongo(self, meta=dict()):
        retrieved = False
        query = {
            "duration.secs": self.time_window.secs
        }
        logs = self._db.query(ProcessMsg._type, query, meta)
        if len(logs) > 0:
            rospy.loginfo("Clearing current poisson distributions...")
            self.poisson = dict()
            self._init_time = rospy.Time.now()
            for log in logs:
                if log[1]['start'] < self._init_time.secs:
                    self._init_time = rospy.Time(log[1]['start'])
                start = datetime.datetime(
                    log[1]['year'], log[0].month, log[0].day,
                    log[0].hour, log[0].minute
                )
                start = rospy.Time(time.mktime(start.timetuple()))
                end = start + self.time_window
                key = "%s-%s" % (start.secs, end.secs)
                self.poisson[key] = self.default_lambda()
                self.poisson[key].scale = log[0].scale
                self.poisson[key].shape = log[0].shape
                self.poisson[key].set_rate(log[0].rate)
            retrieved = True
        if len(logs) > 0:
            rospy.loginfo("%d new poisson distributions are obtained from db..." % len(logs))
        return retrieved


class PeriodicPoissonProcesses(PoissonProcesses):

    def __init__(
        self, time_window=10, minute_increment=1, periodic_cycle=10080,
        coll="poisson_processes"
    ):
        self.periodic_cycle = periodic_cycle
        super(PeriodicPoissonProcesses, self).__init__(time_window, minute_increment, coll)

    def get_lambda_at(self, start_time):
        if self._init_time is None:
            init_time = start_time
        else:
            init_time = self._init_time
        delta = (start_time - init_time).secs % (self.minute_increment * self.periodic_cycle).secs
        start_time = init_time + rospy.Duration(delta, start_time.nsecs)
        return super(PeriodicPoissonProcesses, self).get_lambda_at(start_time)

    def set_lambda_at(self, start_time, lmbd):
        if self._init_time is None:
            init_time = start_time
        else:
            init_time = self._init_time
        delta = (start_time - init_time).secs % (self.minute_increment * self.periodic_cycle).secs
        start_time = init_time + rospy.Duration(delta, start_time.nsecs)
        return super(PeriodicPoissonProcesses, self).set_lambda_at(start_time, lmbd)

    def update(self, start_time, count):
        if self._init_time is not None:
            if start_time < self._init_time:
                self._reconstruct_process(start_time, count)
                return
            delta = (start_time - self._init_time).secs % (self.minute_increment * self.periodic_cycle).secs
            start_time = self._init_time + rospy.Duration(delta, start_time.nsecs)
        super(PeriodicPoissonProcesses, self).update(start_time, count)

    def _reconstruct_process(self, start_time, count):
        rospy.loginfo(
            "New initial time %d, reconstructing poisson process..." % start_time.secs
        )
        poisson = copy.deepcopy(self.poisson)
        self.poisson = dict()
        super(PeriodicPoissonProcesses, self).update(start_time, count)
        for key, rate in poisson.iteritems():
            start_time = rospy.Time(int(key.split("-")[0]))
            delta = (start_time - self._init_time).secs % (self.minute_increment * self.periodic_cycle).secs
            start_time = self._init_time + rospy.Duration(delta, start_time.nsecs)
            end_time = start_time + self.time_window
            key = "%s-%s" % (start_time.secs, end_time.secs)
            self.poisson[key] = rate

    def store_to_mongo(self, meta):
        meta.update({'periodic_cycle': self.periodic_cycle})
        super(PeriodicPoissonProcesses, self).store_to_mongo(meta)

    def _store(self, start_time, meta):
        if 'periodic_cycle' not in meta:
            meta.update({'periodic_cycle': self.periodic_cycle})
        if self._init_time is not None:
            delta = (start_time - self._init_time).secs % (self.minute_increment * self.periodic_cycle).secs
            start_time = self._init_time + rospy.Duration(delta, start_time.nsecs)
        super(PeriodicPoissonProcesses, self)._store(start_time, meta)

    def retrieve_from_mongo(self, meta=dict()):
        meta.update({'periodic_cycle': self.periodic_cycle})
        return super(PeriodicPoissonProcesses, self).retrieve_from_mongo(meta)

    def retrieve(
        self, start_time, end_time, use_upper_confidence=False,
        use_lower_confidence=False, scale=False
    ):
        # convert start_time and end_time to the closest time range (in minutes)
        start_time = super(PeriodicPoissonProcesses, self)._convert_time(start_time)
        end_time = super(PeriodicPoissonProcesses, self)._convert_time(end_time)
        real_start = start_time
        result = dict()
        if self._init_time is not None:
            temp_start_time = start_time
            delta = (start_time - self._init_time).secs % (self.minute_increment * self.periodic_cycle).secs
            start_time = self._init_time + rospy.Duration(delta, start_time.nsecs)
            end_time = start_time + (end_time - temp_start_time)
            while start_time + self.time_window <= end_time:
                conditions = (start_time - self._init_time).secs % (self.minute_increment * self.periodic_cycle).secs == 0
                conditions = conditions and ((start_time - self._init_time).secs != 0)
                if conditions:
                    start_time = start_time - (self.minute_increment * self.periodic_cycle)
                    end_time = end_time - (self.minute_increment * self.periodic_cycle)
                mid_end = start_time + self.time_window
                temp = super(PeriodicPoissonProcesses, self).retrieve(
                    start_time, mid_end, use_upper_confidence,
                    use_lower_confidence, scale
                )
                if len(temp) == 1:
                    key = "%s-%s" % (real_start.secs, (real_start + self.time_window).secs)
                    result.update({key: temp.values()[0]})
                start_time = start_time + self.minute_increment
                real_start = real_start + self.minute_increment
        return result

    def get_one_periodic_rate(self):
        if self._init_time is not None:
            start_time = self._init_time
            end_time = start_time + rospy.Duration(self.periodic_cycle*60)
            end_time = end_time + self.time_window - self.minute_increment
            poisson = self.retrieve(start_time, end_time)
            upper = self.retrieve(start_time, end_time, use_upper_confidence=True)
            lower = self.retrieve(start_time, end_time, use_lower_confidence=True)
            keys = sorted(poisson.keys())
            rates = list()
            upper_limits = list()
            lower_limits = list()
            for key in keys:
                rates.append(poisson[key])
                upper_limits.append(abs(poisson[key] - upper[key]))
                lower_limits.append(abs(poisson[key] - lower[key]))
            return rates, lower_limits, upper_limits
        return list(), list(), list()
