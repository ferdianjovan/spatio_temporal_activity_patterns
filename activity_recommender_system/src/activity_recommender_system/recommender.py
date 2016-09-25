#!/usr/bin/env python


import yaml
import rospy
import roslib
import random
# from std_msgs.msg import String
from region_observation.util import is_intersected
from strands_navigation_msgs.msg import TopologicalMap
from people_temporal_patterns.srv import PeopleEstimateSrv
from strands_exploration_msgs.srv import GetExplorationTasks
from activity_temporal_patterns.srv import ActivityEstimateSrv
from region_observation.util import robot_view_cone, get_soma_info
from strands_exploration_msgs.srv import GetExplorationTasksResponse


class ActivityRecommender(object):

    def __init__(self, name):
        rospy.loginfo("Initiating activity recommender system...")
        rospy.loginfo(
            "Connecting to %s service..." % rospy.get_param(
                "~people_srv", "/people_counter/people_estimate"
            )
        )
        self.people_srv = rospy.ServiceProxy(
            rospy.get_param("~people_srv", "/people_counter/people_estimate"),
            PeopleEstimateSrv
        )
        self.people_srv.wait_for_service()
        rospy.sleep(0.1)
        rospy.loginfo(
            "Connecting to %s service..." % rospy.get_param(
                "~activity_srv", "/activity_counter/activity_estimate"
            )
        )
        self.act_srv = rospy.ServiceProxy(
            rospy.get_param("~activity_srv", "/activity_counter/activity_estimate"),
            ActivityEstimateSrv
        )
        self.act_srv.wait_for_service()
        # self.poisson_consent = PoissonWrapper(
        #     rospy.get_param("~consent_topic", "/skeleton_data/consent_ret"),
        #     String, "data", "nothing", dict(), time_window*3, time_increment, 1440
        # )
        rospy.sleep(0.1)
        self.epsilon = 0.15
        self.topo_map = None
        if rospy.get_param("~with_config_file", False):
            self.region_wps = self._get_waypoints_from_file()
        else:
            self.region_wps = self._get_waypoints(
                rospy.get_param("~soma_config", "activity_exploration")
            )

        rospy.loginfo(
            "Region ids and their nearest waypoints: %s" % str(self.region_wps)
        )
        rospy.sleep(0.1)
        self.service = rospy.Service(
            '%s/activity_rcmd_srv' % rospy.get_name(),
            GetExplorationTasks, self._srv_cb
        )
        rospy.loginfo("%s/activity_rcmd_srv service is ready..." % rospy.get_name())
        rospy.sleep(0.1)

    def _srv_cb(self, msg):
        rospy.loginfo(
            "Got a request to find waypoints to visit between %d and %d"
            % (msg.start_time.secs, msg.end_time.secs)
        )
        people_estimates = self.people_srv(msg.start_time, msg.end_time, False)
        visit_plan = list()
        visit_plan = [
            (estimate, people_estimates.region_ids[ind]) for ind, estimate in enumerate(
                people_estimates.estimates)
        ]
        visit_plan = sorted(visit_plan, key=lambda i: i[0], reverse=True)
        visit_plan = self._check_visit_plan(
            msg.start_time, msg.end_time, visit_plan
        )
        suggested_wps = list()
        suggested_score = list()
        for i in visit_plan:
            if i[1] in self.region_wps:
                if self.region_wps[i[1]] in suggested_wps:
                    suggested_score[
                        suggested_wps.index(self.region_wps[i[1]])
                    ] += i[0]
                else:
                    suggested_wps.append(self.region_wps[i[1]])
                    suggested_score.append(i[0])
            else:
                rospy.loginfo(
                    "No waypoint covers region %s, removing region..." % i[1]
                )
        task = GetExplorationTasksResponse(
            suggested_wps[:5], suggested_score[:5]
        )
        rospy.loginfo("Recommended waypoints to visit: %s" % str(task))
        # task = self._check_consent(msg, task)
        return task

    def _check_visit_plan(self, start_time, end_time, visit_plan):
        scales = self.people_srv(start_time, end_time, True)
        scale_plan = list()
        for ind, scale in enumerate(scales.estimates):
            scale_plan.append((scale, scales.region_ids[ind]))
        scale_plan = sorted(scale_plan, key=lambda i: i[0], reverse=True)
        lower_threshold = scale_plan[0][0] - (self.epsilon * scale_plan[0][0])
        high_visit = list()
        for total_scale, roi in scale_plan:
            if total_scale <= scale_plan[0][0] and total_scale >= lower_threshold:
                high_visit.append(roi)
        p = len(high_visit) / float(len(scales.estimates))
        scale_plan = sorted(scale_plan, key=lambda i: i[0])
        if random.random() > p:
            rospy.loginfo("Changing WayPoints to visit unobserved places...")
            new_visit_plan = list()
            for i in scale_plan:
                for j in visit_plan:
                    if i[1] == j[1]:
                        new_visit_plan.append(j)
                        break
            visit_plan = new_visit_plan
        return visit_plan

    # def _check_consent(self, msg, task):
    #     rates_consent = self.poisson_consent.retrieve_from_to(
    #         msg.start_time, msg.end_time
    #     )
    #     try:
    #         avg = sum(rates_consent.values()) / float(len(rates_consent.values()))
    #     except:
    #         return task
    #     if avg >= rospy.get_param("~consent_rate", 0.8):
    #         rospy.loginfo("Waypoint's order: %s is shuffled" % str(task.task_definition))
    #         random.shuffle(task.task_definition)
    #     return task

    def _topo_map_cb(self, topo_map):
        self.topo_map = topo_map

    def _get_waypoints(self, soma_config):
        region_wps = dict()
        # get regions information
        regions, _ = get_soma_info(soma_config)
        # get waypoint information
        topo_sub = rospy.Subscriber(
            "/topological_map", TopologicalMap, self._topo_map_cb, None, 10
        )
        rospy.loginfo("Getting information from /topological_map...")
        while self.topo_map is None:
            rospy.sleep(0.1)
        topo_sub.unregister()

        for wp in self.topo_map.nodes:
            wp_sight, _ = robot_view_cone(wp.pose)
            intersected_rois = list()
            intersected_regions = list()
            for roi, region in regions.iteritems():
                if is_intersected(wp_sight, region):
                    intersected_regions.append(region)
                    intersected_rois.append(roi)
            for ind, region in enumerate(intersected_regions):
                area = wp_sight.intersection(region).area
                roi = intersected_rois[ind]
                if roi in region_wps:
                    _, area1 = region_wps[roi]
                    if area > area1:
                        region_wps.update({roi: (wp.name, area)})
                else:
                    region_wps.update({roi: (wp.name, area)})
        return {
            roi: tupleoftwo[0] for roi, tupleoftwo in region_wps.iteritems()
        }

    def _get_waypoints_from_file(self):
        roi_wp_hashmap = yaml.load(
            open(
                roslib.packages.get_pkg_dir('activity_recommender_system') + '/config/region_to_wp.yaml',
                'r'
            )
        )
        return roi_wp_hashmap

if __name__ == '__main__':
    rospy.init_node("arms")
    ar = ActivityRecommender(rospy.get_name())
    rospy.spin()
