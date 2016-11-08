#!/usr/bin/env python

import copy
import time
import rospy
import datetime
import threading
from poisson_processes.processes import PeriodicPoissonProcesses


class PoissonWrapper(object):

    def __init__(
        self, topic, topic_msg, topic_attribute, value, not_value=False,
        meta_info=dict(), max_count=None, window=10, increment=1,
        periodic_cycle=10080
    ):
        rospy.loginfo("Initializing poisson wrapper...")
        self._lock = threading.Lock()
        # for each roi create PoissonProcesses
        rospy.loginfo("Time window is %d minute with increment %d minute" % (window, increment))
        temp = datetime.datetime.fromtimestamp(rospy.Time.now().secs)
        temp = datetime.datetime(
            temp.year, temp.month, temp.day, temp.hour, temp.minute, 0
        )
        self._start_time = rospy.Time(time.mktime(temp.timetuple()))
        self._stored_value = list()
        self.max_count = max_count
        self.not_value = not_value
        self.time_window = rospy.Duration(window*60)
        self.time_increment = rospy.Duration(increment*60)
        rospy.loginfo("Creating a periodic cycle every %d minutes" % periodic_cycle)
        self.process = PeriodicPoissonProcesses(window, increment, periodic_cycle)
        rospy.loginfo(
            "Listening to topic %s with type %s, counting attribute %s with value %s" %
            (topic, topic_msg._type, topic_attribute, str(value))
        )
        self.attribute = topic_attribute
        self.value = value
        self.topic = topic
        self.topic_msg = topic_msg
        if isinstance(meta_info, dict):
            self.meta_info = meta_info  # to add info like location etc
        else:
            self.meta_info = dict()
        self.load_from_db()
        rospy.Subscriber(topic, topic_msg, self._cb, None, 10)
        rospy.sleep(0.1)
        self._thread = threading.Thread(target=self._update)
        self._thread.start()

    def _update(self):
        while not rospy.is_shutdown():
            count = 0
            delta = (rospy.Time.now() - self._start_time)
            if len(self._stored_value) > 0 and delta > self.time_window:
                stored_inds = list()
                temp = copy.deepcopy(self._stored_value)
                for i, j in enumerate(temp):
                    if j[0] >= self._start_time and j[0] < self._start_time + self.time_window:
                        count += j[1]
                    if j[0] >= self._start_time + self.time_increment and i not in stored_inds:
                        stored_inds.append(i)
                self.process.update(self._start_time, max(count, self.max_count))
                self._store(self._start_time)
                self._start_time = self._start_time + self.time_increment
                # updating stored_value
                n = len(temp)
                temp = [temp[i] for i in stored_inds]
                # mutex lock
                self._lock.acquire_lock()
                self._stored_value = temp + self._stored_value[n:]
                self._lock.release_lock()
            rospy.sleep(1)

    def _cb(self, msg):
        current_time = rospy.Time.now()
        value = getattr(msg, self.attribute)
        count = 0
        if isinstance(value, (list, tuple)):
            if self.not_value:
                if self.value == list():
                    count = int(len(value) > 0)
                elif isinstance(self.value, (list, tuple)):
                    count = sum([1 for i in value if value not in self.value])
                else:
                    count = sum([1 for i in value if value != self.value])
            else:
                if self.value == list():
                    count = int(len(value) == 0)
                elif isinstance(self.value, (list, tuple)):
                    count = sum([1 for i in value if value in self.value])
                else:
                    count = sum([1 for i in value if value == self.value])
        elif self.value == value:
            if self.not_value:
                if self.value is None:
                    count = int(value is not self.value)
                elif isinstance(self.value, (list, tuple)):
                    count = int(value not in self.value)
                else:
                    count = int(value != self.value)
            else:
                if self.value is None:
                    count = int(value is self.value)
                elif isinstance(self.value, (list, tuple)):
                    count = int(value in self.value)
                else:
                    count = int(value == self.value)
        # mutex lock
        self._lock.acquire_lock()
        self._stored_value.append((current_time, count))
        self._lock.release_lock()

    def _store(self, start_time):
        meta = {
            "type": self.topic_msg._type, "topic": self.topic,
            "recorded_value": self.value, "recorded_attribute": self.attribute,
            "is_not_value": self.not_value, "max_count": self.max_count
        }
        meta.update(self.meta_info)
        self.process._store(start_time, meta)

    def retrieve_from_to(self, start_time, end_time):
        return self.process.retrieve(start_time, end_time)

    def load_from_db(self):
        meta = {
            "type": self.topic_msg._type, "topic": self.topic,
            "recorded_value": self.value, "recorded_attribute": self.attribute,
            "is_not_value": self.not_value, "max_count": self.max_count
        }
        meta.update(self.meta_info)
        self.process.retrieve_from_mongo(meta)

    def store_to_db(self):
        meta = {
            "type": self.topic_msg._type, "topic": self.topic,
            "recorded_value": self.value, "recorded_attribute": self.attribute
            "is_not_value": self.not_value, "max_count": self.max_count
        }
        meta.update(self.meta_info)
        self.process.store_to_mongo(meta)
