#!/usr/bin/env python

import time
import rospy
import datetime
import threading

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from region_observation.msg import Observation
from mongodb_store.message_store import MessageStoreProxy
from region_observation.util import robot_view_cone, get_soma_info, is_intersected


class OnlineRegionObservation(object):

    def __init__(
        self, name, soma_config, time_increment=1, coll="region_observation"
    ):
        rospy.loginfo("Initializing region observation...")
        # set minute increment of robot observation
        if 60 % time_increment != 0:
            rospy.logwarn("The minute increment was not a factor of 60, setting it to 1...")
            time_increment = 1
        self.time_increment = time_increment
        # get map info
        self.regions, self.soma_map = get_soma_info(soma_config)
        self.soma_config = soma_config
        self.intersected_regions = list()
        # draw robot view cone
        self._pub = rospy.Publisher("%s/view_cone" % name, Marker, queue_size=10)
        # publish the result of the observation
        self._pub_reg = rospy.Publisher("%s/observation" % name, Observation, queue_size=10)
        self._msgs = list()
        self._thread = threading.Thread(target=self.publish_msgs)
        # get robot sight
        self._pan_orientation = 0.0
        rospy.loginfo("Subcribe to /ptu/state...")
        rospy.Subscriber("/ptu/state", JointState, self._ptu_cb, None, 10)
        rospy.sleep(0.1)
        self.region_observation_duration = dict()
        # db for RegionObservation
        rospy.loginfo("Create collection db as %s..." % coll)
        self._db = MessageStoreProxy(collection=coll)
        rospy.loginfo("Subcribe to /robot_pose...")
        rospy.Subscriber("/robot_pose", Pose, self._robot_cb, None, 10)

    def _ptu_cb(self, ptu):
        self._pan_orientation = ptu.position[ptu.name.index('pan')]

    def _robot_cb(self, pose):
        robot_sight, arr_robot_sight = robot_view_cone(pose, self._pan_orientation)
        self.draw_view_cone(arr_robot_sight)
        intersected_regions = list()
        for roi, region in self.regions.iteritems():
            if is_intersected(robot_sight, region):
                intersected_regions.append(roi)
        self.intersected_regions = intersected_regions

    def observe(self):
        rospy.loginfo("Starting robot observation...")
        self._thread.start()
        while not rospy.is_shutdown():
            # get the right starting time
            start_time = rospy.Time.now()
            st = datetime.datetime.fromtimestamp(start_time.secs)
            st = datetime.datetime(st.year, st.month, st.day, st.hour, st.minute)
            start_time = rospy.Time(time.mktime(st.timetuple()))
            if st.minute % self.time_increment != 0:
                st = st - datetime.timedelta(
                    minutes=st.minute % self.time_increment, seconds=st.second
                )
                start_time = rospy.Time(time.mktime(st.timetuple()))
            et = st + datetime.timedelta(minutes=self.time_increment)
            end_time = rospy.Time(time.mktime(et.timetuple()))
            # observe the region(s) within time duration
            self._observe(start_time, end_time)
        self._thread.join()

    def _observe(self, start_time, end_time):
        # get where the robot is initially
        prev_roi = self.intersected_regions
        rospy.loginfo("Robot sees regions %s" % (prev_roi))
        roi_start_time = {roi: start_time for roi in prev_roi}
        duration = dict()
        current_time = rospy.Time.now()
        while rospy.Time.now() < end_time:
            if rospy.is_shutdown():
                return
            current_roi = self.intersected_regions
            if len(current_roi) > 0:
                # for a new registered roi, store current time
                for roi in [i for i in current_roi if i not in prev_roi]:
                    rospy.loginfo("Robot sees a new region %s" % roi)
                    roi_start_time[roi] = current_time
                # for a registered roi that was just passed, calculate duration
                for roi in [i for i in prev_roi if i not in current_roi]:
                    rospy.loginfo("Robot leaves region %s" % roi)
                    if roi in duration:
                        duration[roi] += (current_time - roi_start_time[roi])
                    else:
                        duration[roi] = current_time - roi_start_time[roi]
                    del roi_start_time[roi]
            if prev_roi != current_roi:
                prev_roi = current_roi
            current_time = rospy.Time.now()
            rospy.sleep(0.05)

        for roi, st in roi_start_time.iteritems():
            duration[roi] = current_time - st
        # save the result
        self.save_observation(duration, start_time, end_time)

    def save_observation(self, durations, start_time, end_time):
        end_time = end_time - rospy.Duration(0, 1)
        rospy.loginfo(
            "Save observation within %s and %s..." % (str(start_time), str(end_time))
        )
        self._msgs = list()
        for roi, duration in durations.iteritems():
            if duration > rospy.Duration(60):
                duration = rospy.Duration(60)
            msg = Observation(
                self.soma_map, self.soma_config, roi,
                start_time, end_time, duration
            )
            self._db.insert(msg)
            # use this to publish via a topic
            self._msgs.append(msg)

    def publish_msgs(self):
        while not rospy.is_shutdown():
            for msg in self._msgs:
                self._pub_reg.publish(msg)
                rospy.sleep(0.1)
            rospy.sleep(0.1)

    def draw_view_cone(self, view_cone):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "view_cone"
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.g = 1.0
        for ind, point in enumerate(view_cone+[view_cone[0]]):
            if ind == 2:
                marker.points.append(Point(point[0], point[1], 1.65))
            else:
                marker.points.append(Point(point[0], point[1], 0.1))
        self._pub.publish(marker)
