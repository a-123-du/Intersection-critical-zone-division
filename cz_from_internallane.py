from decimal import Decimal
from math import *
import os
from xml.dom.minidom import parse
import numpy as np
from pylab import *
import sympy
import sumolib
import shapely.geometry.linestring
from matplotlib import pyplot as plt
from pylab import xticks, yticks, np
import sys
import math
from shapely.geometry import Point
from shapely.geometry import LineString
from shapely.geometry.polygon import LinearRing
from shapely.geometry import Polygon
import matplotlib.patches as mpatches
from matplotlib.legend_handler import HandlerPatch
from traci_study.parse_xml import Net

count = 0
index = 0
vhe_leni = 0
DSAFE = 0  # Define constants, safety distance
STRAIGHT_DSAFE = 0.6  # Define the safety distance for the critical zone of a straight-line route
WSAFE = 0.6  # Safety width: The sum of vehicle width and safety width is less than the lane width, so set it to 0.6
WVEHICLE = 2  # Vehicle width
LVEHICLE = 5  # Vehicle length
stop_critical_point_distance = 0.1  # Define the safety distance between the critical point and the stopping point
poly_i = []  # poly_i obtains the distance from each point on the polyline to the initial point
cz_index = []  # Store the midpoint along the rear edge of the critical zone, representing the sequence of the critical zone
CZ = []  # Store the coordinates of the vertices of the critical zone
c_and_line = []  # Stored are the intersection points between the line segment and the circle
along_lane = []
lines = []  # Store each polyline segment from the built-in lane to the exit lane
lines_all = []  # Stored includes all polyline segments from the entry lane to the exit lane
point = []  # Store the set of points starting from the first endpoint of the built-in lane (including the points of the built-in lane and the endpoint of the exit lane)
internal_point = []  # Stored are the endpoints of the built-in lane
stop_point_x = []  # The horizontal and vertical coordinates of the stopping point in the critical zone
stop_point_y = []
stop_point = []  # The stopping point in the critical zone
stop_dis_from_start = []  # he driving distance along the built-in lane for each stopping point in the sequence of critical zones
straight_vhe_fc = []  # The front of the vehicle in the straight critical zone
czRP_cz = dict()  # Key: midpoint of the trailing edge of c, value: critical section list


# Divide critical zones from the intersection's built-in lane according to the partitioning rules and calculate the stopping points for the critical zones
# Input: Coordinates of polyline points for the known built-in lane: A(xA, yA), B(xB, yB), C(xC, yC), D(xD, yD), E(xE, yE)
# Output: Obtain the sequence of critical zones and the coordinates of vertices for each critical zone.

# Retrieve the polyline segment
def get_polylines(point_array):
    polyline = LineString(point_array)
    return polyline


# Retrieve the length of each segment of the polyline
def get_polyline_length(point_array):
    i = 0
    while i < len(point_array) - 1:
        polyline = LineString([point_array[i], point_array[i + 1]])
        poly_length[i] = polyline.length
        i += 1
    return


def get_polyi(p_length):  # Calculate the values from the starting endpoint to each endpoint of the polyline segment
    length_i = 0
    i = 0
    while i < len(p_length) - 1:  # Exclude the endpoint of the exit lane from consideration
        length_i += p_length[i]
        poly_i.append(length_i)
        i += 1
    return


# Obtain the values for the safety distance
# The front of the critical zone is P1, P2, and the rear is P3, P4. Compare distances 3-1, 3-2, 4-1, 4-2 (equivalent, choose any two), and select the minimum value as the distance between the front and rear critical zones
def get_new_dsafe(cz_list):
    dis = []  # Two distances
    cz_dis = []  # The distance between each adjacent critical zone
    for i in range(len(cz_list) - 1):
        dis1 = LineString([cz_list[i + 1][3], cz_list[i][0]]).length
        dis2 = LineString([cz_list[i + 1][3], cz_list[i][1]]).length
        cz_dis.append(min(dis1, dis2))
    DSAFE = float('{:.2f}'.format(max(cz_dis)))  # Round to two decimal places
    return DSAFE


# Calculate the intersection points between the polyline segment and the circle (obtain the center of the vehicle's front suspension)
def get_vehicle_frontCenter(c, pointA, pointB, radius):
    l = LineString([pointA, pointB])  # Define a line segment
    circle = c.buffer(radius).boundary  # Define a circle
    i = circle.intersection(l)
    if isinstance(i, shapely.geometry.linestring.LineString):  # The line segment and the circle have no intersection points
        return 0
    elif isinstance(i, shapely.geometry.point.Point):  # The line segment and the circle have one intersection point
        c_and_line.append(i)
        return 1
    elif isinstance(i, shapely.geometry.multipoint.MultiPoint):  # The line segment and the circle have two intersection points. It is necessary to choose the one in the direction of travel and exclude the other point
        # Consider the cases and select the point closer to the endpoint of the polyline segment. Place that point into an array
        pp1 = Point(i.geoms[0].coords[0])
        pp2 = Point(i.geoms[1].coords[0])
        poly1 = LineString([pp1, pointB])
        poly2 = LineString([pp2, pointB])
        if poly1.length < poly2.length:
            c_and_line.append(Point(i.geoms[0].coords[0]))
        else:
            c_and_line.append(Point(i.geoms[1].coords[0]))
        return 2


# Retrieve the list of the front of turning vehicles
def get_turning_vehicle_fclist():
    veh_start = point[0]  # Define the starting point of the vehicle
    for j in range(1, len(point) - 1, 1):
        pa = point[j - 1]  # Define the starting point of the polyline segment
        pb = point[j]  # Define the endpoint of the polyline segment
        rest_len = LineString([veh_start, pb]).length  # Define the remaining length of the polyline segment
        while rest_len >= LVEHICLE:
            get_vehicle_frontCenter(veh_start, pa, pb, LVEHICLE)
            vhe_leni = LineString([c_and_line[-1], pa]).length + poly_i[j - 1]
            along_lane.append(vhe_leni)  # Store the distance each front of the vehicle travels through the lane, which, for turning vehicles, is essentially the distance along the built-in lane from the rear edge of the critical zone
            veh_start = c_and_line[-1]  # Take the elements after appending and update the front of the vehicle
            rest_len = LineString([veh_start, pb]).length
    get_vehicle_frontCenter(veh_start, point[j], point[len(point) - 1], LVEHICLE)  # Handle the last critical zone, part of which is on the exit lane
    return


# Retrieve the list of the front of straight-traveling vehicles. The midpoint for straight vehicles is the vehicle length plus the safety distance, and the front is similar to the vehicle length plus the safety distance
def get_straight_vehicle_fclist(straight_safe_dis):
    veh_start = point[0]  # Define the starting point of the vehicle
    for j in range(1, len(point) - 1, 1):
        pa = point[j - 1]  # Define the starting point of the polyline segment
        pb = point[j]  # Define the endpoint of the polyline segment
        rest_len = LineString([veh_start, pb]).length  # Define the remaining length of the polyline segment
        while rest_len >= LVEHICLE + straight_safe_dis:  # Because we are currently comparing the remaining distance with the vehicle length, for straight-traveling vehicles, the vehicle length is the extended safety distance
            get_vehicle_frontCenter(veh_start, pa, pb, LVEHICLE + straight_safe_dis)
            vhe_leni = LineString([c_and_line[-1], pa]).length + poly_i[j - 1]
            along_lane.append(vhe_leni)  # Store the distance each rear center of the critical zone travels through the lane
            veh_start = c_and_line[-1]  # Take the elements after appending and update the front of the vehicle
            rest_len = LineString([veh_start, pb]).length
    get_vehicle_frontCenter(veh_start, point[j], point[len(point) - 1],
                            LVEHICLE + straight_safe_dis)  # Handle the last critical zone, part of which is on the exit lane
    return


# Obtain the position of the critical zone based on the direction of the vehicle's front and rear suspension centers (axles) with respect to the safety distance and width
def get_cz_frontCenter(pVFCenter, pVRCenter, Dsafe, Wsafe, Wvehicle, lvehicle):
    vetor_F = np.array([pVFCenter.x, pVFCenter.y])
    vetor_R = np.array([pVRCenter.x, pVRCenter.y])
    vetor = vetor_R - vetor_F
    vetor_size = np.linalg.norm(vetor)  # Calculate the magnitude of a vector
    e = vetor / vetor_size  # Calculate the unit vector
    vector2 = Dsafe * e  # The vector from the front of the vehicle to the center of the front edge of the critical zone
    if pVFCenter.x <= pVRCenter.x and pVFCenter.y <= pVRCenter.y:  # If the coordinates of the front are smaller than those of the rear, then it's a subtraction
        cz_fx = pVFCenter.x - vector2[0]  # Obtain the horizontal coordinate of the center of the front edge of the critical zone
        cz_fy = pVFCenter.y - vector2[1]  # Obtain the vertical coordinate of the center of the front edge of the critical zone
    elif pVFCenter.x < pVRCenter.x and pVFCenter.y > pVRCenter.y:  # If the front of the vehicle has a smaller x-coordinate and a larger y-coordinate than the rear of the vehicle
        cz_fx = pVFCenter.x - vector2[0]
        cz_fy = pVFCenter.y - vector2[1]
    elif pVFCenter.x > pVRCenter.x and pVFCenter.y < pVRCenter.y:
        cz_fx = pVFCenter.x - vector2[0]
        cz_fy = pVFCenter.y - vector2[1]
    elif pVFCenter.x >= pVRCenter.x and pVFCenter.y >= pVRCenter.y:
        cz_fx = pVFCenter.x - vector2[0]
        cz_fy = pVFCenter.y - vector2[1]
    cz_f = Point([cz_fx, cz_fy])  # The center point of the front edge of the critical zone
    x = [cz_fx, pVRCenter.x]
    y = [cz_fy, pVRCenter.y]
    # Calculate the deviation in the x-direction and y-direction.
    Lcz = lvehicle + Dsafe  # The length of the critical zone
    Wcz = 0.5 * Wvehicle + Wsafe  # Half of the total width of the critical zone
    sin = abs(cz_fy - pVRCenter.y) / Lcz
    cos = abs(cz_fx - pVRCenter.x) / Lcz
    deta_x = Wcz * sin  # The offset in the x-direction
    deta_y = Wcz * cos  # The offset in the y-direction
    if cos - 0 < 1e-3:  # cos == 0:  # Indicating vertical travel, where deta_y is 0. In reality, it can be used in either situation
        CFp1 = Point([cz_fx + deta_x, cz_fy - deta_y])  # Obtain the two endpoints of the front edge of the critical zone
        CFp2 = Point([cz_fx - deta_x, cz_fy + deta_y])
        CRp3 = Point(pVRCenter.x + deta_x, pVRCenter.y - deta_y)
        CRp4 = Point(pVRCenter.x - deta_x, pVRCenter.y + deta_y)
    else:
        slope, intercept = np.polyfit(x, y, 1)  # If the slope of the critical zone axis exists, calculate the slope of the critical zone axis
        if slope >= 0:  # If the slope is positive, the coordinates of the vertex are of opposite signs. If the slope is negative, the coordinates of the vertex are either added or subtracted simultaneously. For a slope of 0, it can be used in either situation, and detax is 0
            CFp1 = Point([cz_fx + deta_x, cz_fy - deta_y])  # Obtain the two endpoints of the front edge of the critical zone
            CFp2 = Point([cz_fx - deta_x, cz_fy + deta_y])
            CRp3 = Point(pVRCenter.x + deta_x, pVRCenter.y - deta_y)  # Obtain the two endpoints of the rear edge of the critical zone
            CRp4 = Point(pVRCenter.x - deta_x, pVRCenter.y + deta_y)
        elif slope < 0:
            CFp1 = Point([cz_fx + deta_x, cz_fy + deta_y])  # Obtain the two endpoints of the front edge of the critical zone
            CFp2 = Point([cz_fx - deta_x, cz_fy - deta_y])
            CRp3 = Point(pVRCenter.x + deta_x, pVRCenter.y + deta_y)
            CRp4 = Point(pVRCenter.x - deta_x, pVRCenter.y - deta_y)
    CZ.append([CFp1, CFp2, CRp4, CRp3])  # Add the obtained coordinates of the critical zone vertices to the critical zone sequence
    return


def get_cz_rm():  # Obtain the waiting point for each critical zone
    cz_index = [point[0]] + c_and_line[0:len(c_and_line) - 1:1]  # Critical zone labeling, stored as the center of the rear edge of the critical zone
    return cz_index


def point_distance_line(point, line_point1, line_point2):
    # Calculate the vector
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
    return distance


# Retrieve the coordinate sequence of the front suspension center of straight-traveling vehicles. Since the front is extended by the safety distance, subtract it now. Straight travel can be in horizontal or vertical form, so it needs to be determined which type of straight travel it is
def get_straight_vhe_fc():
    vhe_fc_x = []  # Store the x-coordinate of the front suspension of the vehicle
    vhe_fc_y = []  # Store the y-coordinate of the front suspension of the vehicle
    for vhe_fc in c_and_line:  # In 'c_and_line,' it stores the front of the vehicle (of type Point)
        vhe_fc_x.append(vhe_fc.x)
        vhe_fc_y.append(vhe_fc.y)
    x = len(set(vhe_fc_x))
    y = len(set(vhe_fc_y))
    if x > 1:  # Horizontal travel, x-coordinate changes, divided by direction
        if vhe_fc_x[0] < vhe_fc_x[-1]:  # From west to east, in the direction of increasing x-axis
            for vhe_fcx in vhe_fc_x:
                vhe_fcx -= STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fcx, vhe_fc_y[0]))
        elif vhe_fc_x[0] > vhe_fc_x[-1]:  # From east to west, in the direction of decreasing x-axis
            for vhe_fcx in vhe_fc_x:
                vhe_fcx += STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fcx, vhe_fc_y[0]))
    elif y > 1:  # Vertical travel, y-coordinate changes, divided by direction
        if vhe_fc_y[0] < vhe_fc_y[-1]:  # From south to north, in the direction of increasing y-axis
            for vhe_fcy in vhe_fc_y:
                vhe_fcy -= STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fc_x[0], vhe_fcy))
        elif vhe_fc_y[0] > vhe_fc_y[-1]:  # From north to south, in the direction of decreasing y-axis
            for vhe_fcy in vhe_fc_y:
                vhe_fcy += STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fc_x[0], vhe_fcy))
    return straight_vhe_fc


# Calculate the angle between two lines
def GetCrossAngle(line1, line2):
    """
       Calculate the angle between two line segments
       :param line1:[x1,y1,x2,y2]
       :param line2:[x3,y3,x4,y4]
       :return:
       """
    dx1 = line1[0] - line1[2]
    dy1 = line1[1] - line1[3]
    dx2 = line2[0] - line2[2]
    dy2 = line2[1] - line2[3]
    angle1 = math.atan2(dy1, dx1)
    angle1 = int(angle1 * 180 / math.pi)
    angle2 = math.atan2(dy2, dx2)
    angle2 = int(angle2 * 180 / math.pi)
    if angle1 * angle2 >= 0:
        insideAngle = abs(angle1 - angle2)
    else:
        insideAngle = abs(angle1) + abs(angle2)
        if insideAngle > 180:
            insideAngle = 360 - insideAngle
    insideAngle = insideAngle % 180
    return insideAngle


def get_moved_point(pointA, pointB, distance):  # Retrieve the point on the line (A-B) after moving a distance from endpoint A
    circle = pointA.buffer(distance).boundary
    stop_point = circle.intersection(LineString([pointA, pointB]))  # Line3 is the line segment from the rear edge of the critical zone to the starting point of the polyline. Determine the stopping point-critical point intersection
    return stop_point


def get_type_one(cz):  # Is the midpoint of the rear edge of the critical zone in the list of endpoints of the polyline
    rmp_x = (cz[2].x + cz[3].x) * 0.5
    rmp_x = '%.2f' % rmp_x  # Format and round to two decimal places
    rmp_x = float(rmp_x)
    rmp_y = (cz[2].y + cz[3].y) * 0.5
    rmp_y = '%.2f' % rmp_y  # Calculate the coordinates of the center of the rear edge of the critical zone
    rmp_y = float(rmp_y)
    for line in lines_all:
        if line.touches(Point(rmp_x, rmp_y)):  # The center coordinates of the rear edge of the critical zone are at the endpoints of the polyline segment
            return line
    return 0  # Belongs to the second or third case


def get_rmp_line(cz):  # Obtain the polyline segment where the center of the rear edge of the critical zone is located
    distance = []  # Store the distance from the center of the rear edge to each polyline segment, facilitating the identification of the minimum distance
    rmp_x = (cz[2].x + cz[3].x) * 0.5
    rmp_y = (cz[2].y + cz[3].y) * 0.5  # Calculate the coordinates of the center of the rear edge of the critical zone
    cz_rear_midpoint = np.array(Point(rmp_x, rmp_y))  # Converting Point to an array type
    for line in lines:
        Linep2 = np.array(line.coords[1])
        Linep1 = np.array(line.coords[0])
        distance_point_line = point_distance_line(cz_rear_midpoint, Linep1, Linep2)  # Calculate the distance from a point to a line segment
        distance.append(distance_point_line)
    min_dis = distance[0]
    min_index = 0
    for dis_index in range(len(distance)):  # Iterate through distances from a point to all polyline segments, find the polyline segment with the minimum distance; that is the polyline segment where the center of the rear edge is located
        if distance[dis_index] < min_dis:  # Determine if the point is on the polyline segment and find the minimum distance
            min_dis = distance[dis_index]
            min_index = dis_index  # Record the index of the segment with the minimum distance
    return lines[min_index]


# Generate the critical zone sequence based on the array of the front suspension centers of the vehicles
def get_cz_from_vfp(c_and_line, veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE):
    for m in range(len(c_and_line)):  # Iterate through the array of front suspension centers of the vehicles to generate the critical zone
        get_cz_frontCenter(c_and_line[m], veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE)
        veh_r = c_and_line[m]
    return


# Solve for the stopping point corresponding to the first and third cases, where line1 is the polyline segment containing the center of the rear edge
def get_sp_oneAndThree(cz, line1, Wv):
    # First, calculate the distance from a point to a line, the distance along the perpendicular segment
    point = [cz[2].x, cz[2].y]  # Point
    vertex_point1 = [line1.coords[0][0], line1.coords[0][1], line1.coords[1][0], line1.coords[1][1]]  # The first polyline segment
    line_point1, line_point2 = np.array(vertex_point1[0:2]), np.array(vertex_point1[2:])  # Convert it into an array
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    m = np.linalg.norm(line_point1 - line_point2)
    if m == 0:
        print('error')
        return 0
    else:
        distance = np.abs(np.cross(vec1, vec2)) / m  # The distance along the perpendicular segment
    # Calculate the intersection point of two lines as the foot of the perpendicular
    x1, x2 = line1.coords[0][0], line1.coords[1][0]
    y1, y2 = line1.coords[0][1], line1.coords[1][1]
    x3, y3 = cz[2].x, cz[2].y
    x = sympy.Symbol('x')
    y = sympy.Symbol('y')
    res = sympy.solve([(y2 - y1) * x + (x1 - x2) * y + x2 * y1 - x1 * y2,
                       (x2 - x1) * x + (y2 - y1) * y - (x2 - x1) * x3 - (y2 - y1) * y3], x, y)  # The intersection point of the polyline segment and the perpendicular segment
    pedal = Point(res[x], res[y])  # The foot of the perpendicular
    if distance > 0.5 * Wv:  # The front of the vehicle can continue to move forward. Calculate the stopping point
        # Calculate the angle between two lines, line1, and half of the rear edge of the critical zone
        rmp_x = (cz[2].x + cz[3].x) * 0.5
        rmp_y = (cz[2].y + cz[3].y) * 0.5  # Calculate the coordinates of the center of the rear edge of the critical zone
        line2 = point + [rmp_x, rmp_y]  # Half of the rear edge of the critical zone
        angle = GetCrossAngle(vertex_point1, line2)  # The angle between two adjacent line segments
        vertex_stop_dis = 0.5 * Wv / tan(radians(angle))  # The distance from the turning point to the parking point
        circle_p = Point(rmp_x, rmp_y)
        if vertex_stop_dis < 2e-2:  # Due to the limitations in the computer's ability to handle decimal places and the presence of scenarios with extremely close values, an approximate calculation is adopted, taking the center of the rear edge of the critical zone directly as the critical point
            critical_point = circle_p
            stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
            return stop_point
        else:
            circle = circle_p.buffer(vertex_stop_dis).boundary
            critical_point = circle.intersection(
                LineString([circle_p, line1.coords[0]]))  # Line3 is the line segment from the rear edge of the critical zone to the starting point of the polyline. Determine the critical point
            stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
            return stop_point
    else:  # The critical point is the perpendicular foot
        stop_point = get_moved_point(pedal, line1.coords[0], stop_critical_point_distance)
        return stop_point  # The perpendicular foot serves as the critical point


# Obtain the length of the polyline segment before the current point 
def get_last_segments(line):
    last_length = 0
    for i in lines_all:  # Iterate through all lanes and find the current segment
        if i != line:
            last_length += i.length
        else:
            return last_length


# Distinguish between Scenario 3 and Scenario 2
def disting_type_twoThree(cz):
    line = get_rmp_line(cz)  # After the critical region, along the polyline segment where the midpoint is located
    fmp_x = (cz[0].x + cz[1].x) * 0.5
    fmp_y = (cz[0].y + cz[1].y) * 0.5  # The coordinates of the center along the front of the critical region
    # Whether the midpoint of the front is on the straight line of the polyline segment where the midpoint of the back is located. Calculate the distance from the point to the line to determine if it is zero
    point = [fmp_x, fmp_y]  # The midpoint of the front
    vertex_point1 = [line.coords[0][0], line.coords[0][1], line.coords[1][0], line.coords[1][1]]  # The first polyline segment
    line_point1, line_point2 = np.array(vertex_point1[0:2]), np.array(vertex_point1[2:])  # Convert to an array
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    m = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)  # The distance from the midpoint of the front to the straight line where the midpoint of the back is located
    if m - 0 < 0.1:  # Inexact solution
        print('The front midpoint is on the line along the back midpoint, which belongs to case 2')
        return 2
    else:
        print('The front midpoint is not on the line along the back midpoint, which belongs to case 3')
        return 3


# Classification of solutions for Scenario 2 and Scenario 4
def get_div_two_four(cz, line1, Wv, Lv):  # line1 is the polyline segment where the back edge of the critical area is located
    pre_line = LineString([Point(0, 0), Point(1, 1)])
    rmp_x = (cz[2].x + cz[3].x) * 0.5
    rmp_y = (cz[2].y + cz[3].y) * 0.5  # Find the coordinates of the center of the critical area's trailing edge
    rmp = Point(rmp_x, rmp_y)  # trailing edge midpoint
    x = LineString([Point(line1.coords[0]), rmp]).length  # GE, the distance between the inflection point and the midpoint of the trailing edge of the critical area
    last_dis = LineString([line1.coords[0], rmp]).length  # The distance from the starting point of the polyline segment to the midpoint of the trailing edge of the critical area
    if abs(last_dis - Lv) < 2e-2 or last_dis - Lv > 0:  # l If the remaining length is greater than or equal to Lv, then the critical point is the midpoint of the trailing edge of the critical area,
        stop_point = get_moved_point(rmp, line1.coords[0], stop_critical_point_distance)
        return stop_point
    else:  # If the remaining length is less than Lv, then there will be two situations, one of which is very small
        for i in range(len(lines) - 1):
            if i < len(lines) and lines[i + 1] == line1:
                pre_line = lines[i]  # pre_line is the previous polyline segment (the one that passes through first) of the polyline segment where the rear edge of the critical section is located.
        pre_line_p = [pre_line.coords[0][0], pre_line.coords[0][1], pre_line.coords[1][0],
                      pre_line.coords[1][1]]
        cur_line_p = [line1.coords[0][0], line1.coords[0][1], line1.coords[1][0], line1.coords[1][1]]  # Current polyline segment
        beta = radians(GetCrossAngle(cur_line_p, pre_line_p))  # The angle between the front and rear polyline segments is converted into radians.
        if x <= 0.5 * Wv * sin(beta):  # Very small cases, special cases (negative solutions)
            stop_point, line = get_sp_four(Lv, Wv, beta, x, line1)  # Returns the stopping point and the segment it is located in
            if line.distance(stop_point) < 1e-2:
                print("The stop point is on the segment behind the critical region and before the midpoint")
            else:
                print("The stop point is not on the segment behind the critical region and before the midpoint")
            return stop_point, line
        else:  # Non-special solution, case ②
            stop_point, line = get_sp_two(Lv, Wv, beta, x, line1)  # Returns the stopping point and the segment it is located in
            if line.distance(stop_point) < 1e-2:
                print("The stopping point is on the segment behind the critical region along the midpoint")
            else:
                print("The stopping point is not on the segment behind the critical region along the midpoint")
            return stop_point, line


# Stop point for case 2
def get_sp_two(Lv, Wv, beta, x, line1):
    p1 = Point(line1.coords[0])  # starting endpoint
    c1 = Lv * Wv * math.sin(beta) * math.sqrt(
        4 * math.pow(Lv, 2) - 4 * Lv * Wv * math.cos(beta) * math.sin(beta) + math.pow(Wv, 2) * math.pow(math.sin(beta),
                                                                                                         2) + 4 * math.pow(
            x, 2) * math.pow(math.cos(beta), 2) - 4 * math.pow(x, 2))
    c2 = 4 * math.pow(Lv, 2) - 4 * math.cos(beta) * Lv * Wv * math.sin(beta) + math.pow(Wv, 2) * math.pow(
        math.sin(beta), 2)
    res2 = (4 * math.pow(Lv, 2) * x - c1 - 2 * Lv * Wv * x * math.cos(beta) * math.sin(beta)) / c2
    res1 = (4 * math.pow(Lv, 2) * x + c1 - 2 * Lv * Wv * x * math.cos(beta) * math.sin(beta)) / c2
    if res2 < x:  # res2 is GQ, the stopping point can be found based on G
        critical_point = get_moved_point(p1, line1.coords[1], res2)
        stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
        return [stop_point, line1] # In the second case, the segment where the critical section is extended is also returned. In order to be consistent with the fourth special case, the return format is
    elif res1 < x:  # res1 is GQ, the stopping point can be found based on G
        critical_point = get_moved_point(p1, line1.coords[1], res1)
        stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
        return [stop_point, line1]


# Situation four
def get_sp_four(Lv, Wv, beta, x, line1):
    for i in range(len(lines)):
        if i < len(lines) and lines[i + 1] == line1:
            pre_line = lines[i]  # pre_line is the previous polyline segment (the one that passes through first) of the polyline segment where the rear edge of the critical section is located.
    dis = 0.5 * Wv * tan(beta) - x / cos(beta)  # GS is dis, find the S coordinate from G
    G = line1.coords[0]  # starting endpoint
    circle = G.buffer(dis).boundary
    critical_point = circle.intersection(pre_line)  # Find the critical point
    stop_point = get_moved_point(critical_point, pre_line.coords[0], stop_critical_point_distance)  # Find the stopping point
    return [stop_point, pre_line]  # Because in the fourth special case, when calculating the driving distance of the stop point, the segment before the segment where the midpoint of the trailing edge of the critical area is located is used, so this segment needs to be obtained


# Custom legend shape
def make_legend_arrow(legend, orig_handle,
                      xdescent, ydescent,
                      width, height, fontsize):
    p = mpatches.FancyArrow(0, 0.5 * height, width, 0, length_includes_head=True, head_width=0.75 * height,
                            color='black')
    return p


# Get the stopping point of the critical section
def get_cz_stop_point():
    for cz_i in CZ:
        line_one = get_type_one(cz_i)  # Get the type of critical section, indicating that the midpoint of the trailing edge of the critical section is at the endpoint
        if line_one:
            print("The midpoint behind the critical region is at the end of the multi-segment line, which belongs to case 1")
            stop = get_sp_oneAndThree(cz_i, line_one, WVEHICLE)  # Obtain the stopping point when the midpoint of the trailing edge of the critical area is at the endpoint of the polyline
            last_length = get_last_segments(line_one)  # Get the length sum of the segments before the current segment
            stop_along_lane = LineString([stop, line_one.coords[0]]).length + last_length  # The distance between the stop point and the starting point of the current built-in lane
            stop_dis_from_start.append(stop_along_lane)
            if line_one.distance(stop) < 1e-2:
                print("The stopping point is on the segment behind the critical region along the midpoint")
            else:
                print("The stopping point is not on the segment behind the critical region along the midpoint")
        else:
            index = disting_type_twoThree(cz_i)  # Distinguish whether it belongs to situation two or three
            line_twTh = get_rmp_line(cz_i)  # Obtain the polyline segment where the midpoint of the trailing edge of the critical area is located
            if index == 3:
                stop = get_sp_oneAndThree(cz_i, line_twTh, WVEHICLE)  # Get the stopping point of case three
                last_length = get_last_segments(line_twTh)  # Get the length sum of the segments before the current segment
                stop_along_lane = LineString([stop, line_twTh.coords[0]]).length + last_length  # The distance between the stop point and the starting point of the current built-in lane
                stop_dis_from_start.append(stop_along_lane)
                if line_twTh.distance(stop) < 1e-2:
                    print("The stopping point is on the segment behind the critical region along the midpoint")
                else:
                    print("The stopping point is not on the segment behind the critical region along the midpoint")
            elif index == 2:
                stop,line_stop = get_div_two_four(cz_i, line_twTh, WVEHICLE, LVEHICLE)  # Obtain the stopping point of case 2 or 4, and the segment where the stopping point is located
                last_length = get_last_segments(line_twTh)  # Get the length sum of the segments before the current segment
                stop_along_lane = LineString([stop, line_stop.coords[0]]).length + last_length  # The distance between the stop point and the starting point of the current built-in lane
                stop_dis_from_start.append(stop_along_lane)
        stop_point_x.append(stop.x)
        stop_point_y.append(stop.y)
        stop_point.append(stop)
        print("No." + str(CZ.index(cz_i) + 1) + "Critical area stop point coordinates:", stop)
    return

# Draw an image of the critical region and its stopping point
def plot_cz_sp_figure():
    # draw image
    f = plt.figure(figsize=(10, 10))
    stopp = plt.scatter(stop_point_x, stop_point_y, s=3 * 3, c='k', alpha=0.9, linewidths=3, label='StopPoint')
    for sp in stop_point:
        plt.text(round(sp.x, 3), round(sp.y, 3), (round(sp.x, 3), round(sp.y, 3)), ha='left', va='bottom',
                 fontsize=12)
    x1, y1 = polyline_all.xy
    plt.plot(x1, y1)
    ax = plt.gca()
    for a in range(len(CZ)):
        polygon1 = Polygon(CZ[a])
        x2, y2 = polygon1.exterior.xy
        plt.plot(x2, y2)
    red_patch = mpatches.Patch(color='black', label='CriticalZone', linewidth='1.2', fill=False)  # Create a rectangular legend
    # Set directions and add arrows to built-in lanes
    for k in range(len(internal_point) - 1):
        plt.arrow(internal_point[k].x, internal_point[k].y, internal_point[k + 1].x - internal_point[k].x,
                  internal_point[k + 1].y - internal_point[k].y,
                  head_length=0.6, length_includes_head=True,  # The increased length includes the arrow part
                  head_width=0.4, label='Direction', fc='b', ec='b')
    # Set legend
    arrow = plt.arrow(0, 0, 0.5, 0.6, label='Direction', color='black')
    plt.legend([red_patch, stopp, arrow], ['CriticalZone', 'StopPoint', 'Direction'],
               handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow),
                            }, loc='best', fontsize='8', facecolor='orange')
    # It is necessary to set the value range of plt in order to eliminate the problem of too small images caused by too long lengths of entering and exiting lanes.
    x_min = min(internal_line.xy[0]) - 1.5 * LVEHICLE
    y_min = min(internal_line.xy[1]) - 1.5 * LVEHICLE
    x_max = max(internal_line.xy[0]) + 1.5 * LVEHICLE
    y_max = max(internal_line.xy[1]) + 1.5 * LVEHICLE
    plt.xlim((x_min, x_max))
    plt.ylim((y_min, y_max))
    ax.set_aspect(1)  # Make the unit lengths of the xy axis equal
    plt.xticks(size=15)  # Set the size of the xy axis text
    plt.yticks(size=15)
    plt.title("Conflict zone demo", fontsize=25)
    plt.xlabel("X", fontsize=20)
    plt.ylabel("Y", fontsize=20)
    plt.show()
    return


if __name__ == '__main__':
    # ---Parse the road network file and obtain the point coordinates in the shape value
    # net_xml = Net("C:/Users/86188/Desktop/example/single_lane.net.xml")
    net_xml = Net("D:/CyberCarSimulation_SUMO/SumoCurveLane/sumo_net_curve/merge614.net.xml")
    con_matrix = net_xml.conflict_matrix()  # Analyze the road network and obtain the conflict matrix of the road network
    print("The conflict matrix of the road network is ：\n", con_matrix)
    from_lane = input('FromLane:')  # Define entry edge
    to_lane = input('ToLane:')  # Define exit edge
    changed_laneid = net_xml.change_laneid(from_lane, to_lane)  # Convert id form
    shape = net_xml.get_shape(changed_laneid, from_lane, to_lane)  # What is returned is a list of shape strings of built-in lanes, consistent with xml
    POINT_NUM = len(shape)  # The number of end points, turning left and right is 7 points, going straight is 4 points
    POLYLINE_NUM = POINT_NUM - 1  # Number of polyline segments
    point_all = [0] * POINT_NUM  # What is stored is the point set of entry and exit lanes and built-in lanes.
    poly_length = [0] * POLYLINE_NUM  # A list that stores the length of each polyline segment
    for string in shape:
        s2 = string.split(',')
        point_all[count] = Point(float(s2[0]), float(s2[1]))
        count += 1
        if count == len(shape):
            break
    for i in point_all[1::]:  # Contains only endpoints of built-in lanes and output lanes
        point.append(i)
    for i in point_all[1:len(point_all) - 1:1]:
        internal_point.append(i)
    internal_line = get_polylines(internal_point)  # built-in lane segment
    polyline_all = get_polylines(point_all)  # total length of all segment lines
    polyline = get_polylines(point)  # Polyline length of inbound and outbound lanes
    for i in range(len(point) - 1):
        l = LineString([point[i], point[i + 1]])
        lines.append(l)  # lines contains only built-in lane polyline segments
    for i in range(len(point_all) - 1):  # lines_all is all polyline segments
        l = LineString([point_all[i], point_all[i + 1]])
        lines_all.append(l)
    get_polyline_length(point)
    get_polyi(poly_length)  # Gets the values ​​from the first endpoint of the built-in lane to the following endpoints
    poly_i = [0] + poly_i  # poly_i gets the distance from each point of the polyline segment to the initial point
    if len(internal_point) == 2:  # The built-in lane endpoint of the critical section for straight travel is 2
        # If it is going straight, call this function to calculate the head of the car and calculate the critical section
        get_straight_vehicle_fclist(STRAIGHT_DSAFE)  # For straight driving, the vehicle length plus the safety distance is used as the new vehicle length, so the front of the vehicle is after the safety distance has been added
        veh_r = point[0]  # initialization
        get_cz_from_vfp(c_and_line, veh_r, 0, WSAFE, WVEHICLE, LVEHICLE + STRAIGHT_DSAFE)  # dsafe=0 when traveling straight, and the vehicle length has increased the safety distance
        get_straight_vhe_fc()
        for i in straight_vhe_fc:
            print(i)
    else:
        # If it is a turn, call this function to calculate the heading
        get_turning_vehicle_fclist()
        veh_r = point[0]  # initialization
        get_cz_from_vfp(c_and_line, veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE)  # Obtain critical area division without safety distance
        DSAFE = get_new_dsafe(CZ)  # Get the minimum safe distance to eliminate white gaps
        # DSAFE=0
        CZ = []  # Update critical section sequence
        get_cz_from_vfp(c_and_line, veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE)  # Obtain critical area division without safety distance
    along_lane = [0] + along_lane  # The midpoint of the trailing edge of the first critical section is the starting point, which is 0
    print("\n")
    cz_index = get_cz_rm()
    #get_cz_stop_point()  # Get critical section stopping point
    for i in range(1, len(stop_dis_from_start), 1):
        stop_dis_from_start[i] -= lines_all[0].length  # Calculate the distance along the initial point of the input lane. However, at the intersection, it starts from the starting point of the built-in lane. Therefore, from the time you enter the built-in lane, subtract the distance of the starting lane from the distance

    plot_cz_sp_figure()  # Drawing
