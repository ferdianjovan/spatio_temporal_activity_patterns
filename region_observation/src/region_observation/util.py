#!/usr/bin/env python

import math
import rospy
import datetime
import itertools
import numpy as np
from tf.transformations import euler_from_quaternion
from shapely.geometry import Polygon, LineString, Point

from soma_map_manager.srv import MapInfo
from soma_manager.srv import SOMAQueryROIs


# calculate polygon area using Shoelace Formula
# http://stackoverflow.com/questions/24467972/calculate-area-of-polygon-given-x-y-coordinates
def poly_area(x, y):
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def is_intersected(poly1, poly2):
    return poly1.intersects(poly2)


def create_polygon(xs, ys):
    # if poly_area(np.array(xs), np.array(ys)) == 0.0:
    if Polygon(np.array(zip(xs, ys))).area == 0.0:
        xs = [
            [xs[0]] + list(i) for i in itertools.permutations(xs[1:])
        ]
        ys = [
            [ys[0]] + list(i) for i in itertools.permutations(ys[1:])
        ]
        areas = list()
        for ind in range(len(xs)):
            # areas.append(poly_area(np.array(xs[ind]), np.array(ys[ind])))
            areas.append(Polygon(np.array(zip(xs[ind], ys[ind]))))
        return Polygon(
            np.array(zip(xs[areas.index(max(areas))], ys[areas.index(max(areas))]))
        )
    else:
        return Polygon(np.array(zip(xs, ys)))


def create_line_string(points):
    if len(points) < 1:
        return None
    elif len(points) == 1:
        return Point(points[0])
    elif isinstance(points[0], list):
        return LineString(points)
    else:
        return Point(points)


def robot_view_cone(pose, pan_orientation=0.0):
    """
        let's call the triangle PLR, where P is the robot pose,
        L the left vertex, R the right vertex
    """
    d = 4       # max monitored distance: reasonably not more than 3.5-4m
    alpha = 1   # field of view: 57 deg kinect, 58 xtion, we can use exactly 1 rad (=57.3 deg)
    _, _, yaw = euler_from_quaternion(
        [0, 0, pose.orientation.z, pose.orientation.w]
    )
    yaw = (yaw + pan_orientation) % (2 * math.pi)
    lyaw = (yaw - (0.5 * alpha)) % (2 * math.pi)
    ryaw = (yaw + (0.5 * alpha)) % (2 * math.pi)

    Px = pose.position.x
    Py = pose.position.y
    Lx = Px + d * math.cos(lyaw)
    Ly = Py + d * math.sin(lyaw)
    Rx = Px + d * math.cos(ryaw)
    Ry = Py + d * math.sin(ryaw)

    return Polygon([[Lx, Ly], [Rx, Ry], [Px, Py]]), [[Lx, Ly], [Rx, Ry], [Px, Py]]


def robot_view_area(Px, Py, yaw):
    d = 3
    poses = list()
    degree = 45 / float(180) * math.pi

    for i in range(8):
        x = Px + d * (math.cos((yaw+(i * degree) % (2 * math.pi))))
        y = Py + d * (math.sin((yaw+(i * degree) % (2 * math.pi))))
        poses.append([x, y])
    # return [[Px + 4, Py], [Px, Py - 4], [Px - 4, Py], [Px, Py + 4]]
    return poses


def get_dict_observation(msg):
    start = datetime.datetime.fromtimestamp(msg.start_from.secs)
    end = msg.until + rospy.Duration(0, 1)
    end = datetime.datetime.fromtimestamp(end.secs)
    key = "%s-%s" % (start.minute, end.minute)
    result = {
        msg.region_id: {
            start.month: {
                start.day: {start.hour: {key: msg.duration}}
            }
        }
    }
    return result


def get_soma_info(soma_config):
    soma_service = rospy.ServiceProxy("/soma/map_info", MapInfo)
    soma_service.wait_for_service()
    soma_map = soma_service(1).map_name
    rospy.loginfo("Got soma map name %s..." % soma_map)
    # get region information from soma2
    soma_service = rospy.ServiceProxy("/soma/query_rois", SOMAQueryROIs)
    soma_service.wait_for_service()
    result = soma_service(
        query_type=0, roiconfigs=[soma_config], returnmostrecent=True
    )
    # create polygon for each regions
    regions = dict()
    for region in result.rois:
        if region.config == soma_config and region.map_name == soma_map:
            xs = [pose.position.x for pose in region.posearray.poses]
            ys = [pose.position.y for pose in region.posearray.poses]
            regions[region.id] = create_polygon(xs, ys)
    rospy.loginfo("Total regions for configuration %s are %d" % (soma_config, len(regions.values())))
    return regions, soma_map


def get_largest_intersected_regions(reference_region, target_regions):
    areas = list()
    for region in target_regions:
        areas.append(reference_region.intersection(region).area)
    return target_regions[areas.index(max(areas))]
