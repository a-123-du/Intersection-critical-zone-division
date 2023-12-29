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
DSAFE = 0  # 定义常量，安全距离
STRAIGHT_DSAFE = 0.6  # 定义直行路线的临界区的安全距离
WSAFE = 0.6  # 安全宽度，车宽加安全宽度小于车道宽得，所以取0.6
WVEHICLE = 2  # 车辆宽度
LVEHICLE = 5  # 车辆长度
stop_critical_point_distance = 0.1  # 定义临界点和停止点之间的安全距离
poly_i = []  # poly_i得到折线段每个点到初始点的距离
cz_index = []  # 存储临界区后沿的中点，表示临界区的序列
CZ = []  # 存储临界区的顶点坐标
c_and_line = []  # 存储的是线段和圆的交点
along_lane = []
lines = []  # 存储内置车道到驶出车道的每一段折线段
lines_all = []  # 存储的包括驶入车道到驶出车道的所有折线段
point = []  # 存储从内置车道第一个端点开始的点集（内置车道点加驶出车道的端点）
internal_point = []  # 存储的是内置车道的端点
stop_point_x = []  # 临界区的停止点横纵坐标
stop_point_y = []
stop_point = []  # 临界区的停止点
stop_dis_from_start = []  # 临界区序列的每个停止点沿着所在内置车道的行驶距离
straight_vhe_fc = []  # 直行临界区的车头
czRP_cz = dict()  # key:c后沿中点，value:临界区列表


# 从路口内置车道按照划分规则划分临界区,及计算临界区的停止点
# 输入：已知内置车道的折线点的坐标：A(xA,yA)、B(xB,yB)、C(xC、yC)、D(xD、yD)、E(xE、yE)
# 输出：得到临界区序列以及各个临界区的顶点坐标

# 获取折线段
def get_polylines(point_array):
    polyline = LineString(point_array)
    return polyline


# 获取折线段每段的长度
def get_polyline_length(point_array):
    i = 0
    while i < len(point_array) - 1:
        polyline = LineString([point_array[i], point_array[i + 1]])
        print("第" + str(i + 1) + "段折线段的长度为:")
        print(polyline.length)
        poly_length[i] = polyline.length
        i += 1
    return


def get_polyi(p_length):  # 求得从起始端点到每个折线段端点的值
    length_i = 0
    i = 0
    while i < len(p_length) - 1:  # 不考虑驶出车道的那个端点
        length_i += p_length[i]
        poly_i.append(length_i)
        i += 1
    return


# 获取安全距离取值
# 临界区前沿是P1,P2 后沿是P3 P4，比较3-1 3-2 4-1 4-2距离（全等，取两个即可），选取最小的值作为前后临界区间的距离
def get_new_dsafe(cz_list):
    dis = []  # 两个距离
    cz_dis = []  # 每个相邻临界区之间的距离
    for i in range(len(cz_list) - 1):
        dis1 = LineString([cz_list[i + 1][3], cz_list[i][0]]).length
        dis2 = LineString([cz_list[i + 1][3], cz_list[i][1]]).length
        cz_dis.append(min(dis1, dis2))
    DSAFE = float('{:.2f}'.format(max(cz_dis)))  # 保留两位小数
    return DSAFE


# 计算折线段与圆的交点（获得汽车前悬中心）
def get_vehicle_frontCenter(c, pointA, pointB, radius):
    l = LineString([pointA, pointB])  # 定义线段
    circle = c.buffer(radius).boundary  # 定义圆
    i = circle.intersection(l)
    if isinstance(i, shapely.geometry.linestring.LineString):  # 线段和圆没交点
        return 0
    elif isinstance(i, shapely.geometry.point.Point):  # 线段与圆存在1个交点
        c_and_line.append(i)
        return 1
    elif isinstance(i, shapely.geometry.multipoint.MultiPoint):  # 线段与圆存在2个交点，需要选择前进方向的，排除另外一个点
        # 分情况，选离终点折线段端点距离小的那个点，放进数组
        pp1 = Point(i.geoms[0].coords[0])
        pp2 = Point(i.geoms[1].coords[0])
        poly1 = LineString([pp1, pointB])
        poly2 = LineString([pp2, pointB])
        if poly1.length < poly2.length:
            c_and_line.append(Point(i.geoms[0].coords[0]))
        else:
            c_and_line.append(Point(i.geoms[1].coords[0]))
        return 2


# 获取转弯车辆的车头列表
def get_turning_vehicle_fclist():
    veh_start = point[0]  # 定义车辆起点
    for j in range(1, len(point) - 1, 1):
        pa = point[j - 1]  # 定义折线段起始点
        pb = point[j]  # 定义折线段终点
        rest_len = LineString([veh_start, pb]).length  # 定义折线段剩余的长度
        while rest_len >= LVEHICLE:
            get_vehicle_frontCenter(veh_start, pa, pb, LVEHICLE)
            vhe_leni = LineString([c_and_line[-1], pa]).length + poly_i[j - 1]
            along_lane.append(vhe_leni)  # 存储每个车头经过车道的距离，对于转弯车辆来说其实就是临界区后沿在内置车道上的距离
            veh_start = c_and_line[-1]  # 取append后的元素，更新车头
            rest_len = LineString([veh_start, pb]).length
    get_vehicle_frontCenter(veh_start, point[j], point[len(point) - 1], LVEHICLE)  # 处理最后一个临界区，一部分在驶出车道上
    return


# 获取直行车辆的车头列表，直行的中间点是车长＋安全距离，车头类似于是车长加上安全距离后的
def get_straight_vehicle_fclist(straight_safe_dis):
    veh_start = point[0]  # 定义车辆起点
    for j in range(1, len(point) - 1, 1):
        pa = point[j - 1]  # 定义折线段起始点
        pb = point[j]  # 定义折线段终点
        rest_len = LineString([veh_start, pb]).length  # 定义折线段剩余的长度
        while rest_len >= LVEHICLE + straight_safe_dis:  # 因为现在是比较剩余距离和车长，直行的车长是扩展安全距离后的
            get_vehicle_frontCenter(veh_start, pa, pb, LVEHICLE + straight_safe_dis)
            vhe_leni = LineString([c_and_line[-1], pa]).length + poly_i[j - 1]
            along_lane.append(vhe_leni)  # 存储每个临界区后沿中心经过车道的距离
            veh_start = c_and_line[-1]  # 取append后的元素，更新车头
            rest_len = LineString([veh_start, pb]).length
    get_vehicle_frontCenter(veh_start, point[j], point[len(point) - 1],
                            LVEHICLE + straight_safe_dis)  # 处理最后一个临界区，一部分在驶出车道上
    return


# 根据车辆前悬中心和后悬中心（车轴）方向与安全距离和宽度获得临界区的位置
def get_cz_frontCenter(pVFCenter, pVRCenter, Dsafe, Wsafe, Wvehicle, lvehicle):
    vetor_F = np.array([pVFCenter.x, pVFCenter.y])
    vetor_R = np.array([pVRCenter.x, pVRCenter.y])
    vetor = vetor_R - vetor_F
    vetor_size = np.linalg.norm(vetor)  # 求向量的模
    e = vetor / vetor_size  # 求单位向量
    vector2 = Dsafe * e  # 从车头到临界区前沿中心的向量
    if pVFCenter.x <= pVRCenter.x and pVFCenter.y <= pVRCenter.y:  # 车头小于车尾的横纵坐标，那么是减
        cz_fx = pVFCenter.x - vector2[0]  # 求得临界区前沿中心点横坐标
        cz_fy = pVFCenter.y - vector2[1]  # 求得临界区前沿中心点纵坐标
    elif pVFCenter.x < pVRCenter.x and pVFCenter.y > pVRCenter.y:  # 车头小于车位x,车头y大于车尾y
        cz_fx = pVFCenter.x - vector2[0]
        cz_fy = pVFCenter.y - vector2[1]
    elif pVFCenter.x > pVRCenter.x and pVFCenter.y < pVRCenter.y:
        cz_fx = pVFCenter.x - vector2[0]
        cz_fy = pVFCenter.y - vector2[1]
    elif pVFCenter.x >= pVRCenter.x and pVFCenter.y >= pVRCenter.y:
        cz_fx = pVFCenter.x - vector2[0]
        cz_fy = pVFCenter.y - vector2[1]
    cz_f = Point([cz_fx, cz_fy])  # 临界区前沿中心点
    x = [cz_fx, pVRCenter.x]
    y = [cz_fy, pVRCenter.y]
    # 求x方向上、y方向上的偏差
    Lcz = lvehicle + Dsafe  # 临界区长度
    Wcz = 0.5 * Wvehicle + Wsafe  # 临界区总宽度的一半
    sin = abs(cz_fy - pVRCenter.y) / Lcz
    cos = abs(cz_fx - pVRCenter.x) / Lcz
    deta_x = Wcz * sin  # x方向上的偏移量
    deta_y = Wcz * cos  # y方向上的偏移量
    if cos - 0 < 1e-3:  # cos == 0:  # 说明是垂直行驶，deta_y是0，实际上放在哪种情况下都可以
        CFp1 = Point([cz_fx + deta_x, cz_fy - deta_y])  # 求得临界区的前沿的两个端点
        CFp2 = Point([cz_fx - deta_x, cz_fy + deta_y])
        CRp3 = Point(pVRCenter.x + deta_x, pVRCenter.y - deta_y)
        CRp4 = Point(pVRCenter.x - deta_x, pVRCenter.y + deta_y)
    else:
        slope, intercept = np.polyfit(x, y, 1)  # 临界区轴的斜率存在，求出临界区轴的斜率
        if slope >= 0:  # 斜率为正，顶点坐标异号，斜率为负，顶点坐标同时相加减,斜率为0实际上放在哪种情况下都可以，detax是0.
            CFp1 = Point([cz_fx + deta_x, cz_fy - deta_y])  # 求得临界区的前沿的两个端点
            CFp2 = Point([cz_fx - deta_x, cz_fy + deta_y])
            CRp3 = Point(pVRCenter.x + deta_x, pVRCenter.y - deta_y)  # 求得临界区的后沿的两个端点
            CRp4 = Point(pVRCenter.x - deta_x, pVRCenter.y + deta_y)
        elif slope < 0:
            CFp1 = Point([cz_fx + deta_x, cz_fy + deta_y])  # 求得临界区的前沿的两个端点
            CFp2 = Point([cz_fx - deta_x, cz_fy - deta_y])
            CRp3 = Point(pVRCenter.x + deta_x, pVRCenter.y + deta_y)
            CRp4 = Point(pVRCenter.x - deta_x, pVRCenter.y - deta_y)
    CZ.append([CFp1, CFp2, CRp4, CRp3])  # 将求出的临界区顶点坐标加入到临界区序列
    return


def get_cz_rm():  # 获得每个临界区的等待点
    cz_index = [point[0]] + c_and_line[0:len(c_and_line) - 1:1]  # 临界区标号，存储的是临界区的后沿中心
    return cz_index


def point_distance_line(point, line_point1, line_point2):
    # 计算向量
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    distance = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)
    return distance


# 获取直行车辆的前悬中心坐标序列,因为车头是加上了安全距离后的，所以现在需要减去，直行分为水平和竖直形式，所以需要判断是哪种直行
def get_straight_vhe_fc():
    vhe_fc_x = []  # 存储车辆前悬的x坐标
    vhe_fc_y = []  # 存储车辆前悬的y坐标
    for vhe_fc in c_and_line:  # c_and_line中存储的是车头（Point类型）
        vhe_fc_x.append(vhe_fc.x)
        vhe_fc_y.append(vhe_fc.y)
    x = len(set(vhe_fc_x))
    y = len(set(vhe_fc_y))
    if x > 1:  # 水平行驶, x坐标变化,分方向
        if vhe_fc_x[0] < vhe_fc_x[-1]:  # 从西向东，x轴增大方向
            for vhe_fcx in vhe_fc_x:
                vhe_fcx -= STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fcx, vhe_fc_y[0]))
        elif vhe_fc_x[0] > vhe_fc_x[-1]:  # 从东向西，x轴减小方向
            for vhe_fcx in vhe_fc_x:
                vhe_fcx += STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fcx, vhe_fc_y[0]))
    elif y > 1:  # 竖直行驶,y坐标变化，分方向
        if vhe_fc_y[0] < vhe_fc_y[-1]:  # 从南向北，y轴增大方向
            for vhe_fcy in vhe_fc_y:
                vhe_fcy -= STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fc_x[0], vhe_fcy))
        elif vhe_fc_y[0] > vhe_fc_y[-1]:  # 从北向南，y轴减小方向
            for vhe_fcy in vhe_fc_y:
                vhe_fcy += STRAIGHT_DSAFE
                straight_vhe_fc.append(Point(vhe_fc_x[0], vhe_fcy))
    return straight_vhe_fc


# 计算两直线的夹角
def GetCrossAngle(line1, line2):
    """
       计算两条线段之间的夹角
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


def get_moved_point(pointA, pointB, distance):  # 获取在Line上（A-B）从端点A移动distance距离后的点
    circle = pointA.buffer(distance).boundary
    stop_point = circle.intersection(LineString([pointA, pointB]))  # line3是从临界区后沿到折线段初始点的线段,求出停止点临界点
    return stop_point


def get_type_one(cz):  # 临界区的后沿中点是否在折线段的端点列表
    rmp_x = (cz[2].x + cz[3].x) * 0.5
    rmp_x = '%.2f' % rmp_x  # 格式化，保留两位小数
    rmp_x = float(rmp_x)
    rmp_y = (cz[2].y + cz[3].y) * 0.5
    rmp_y = '%.2f' % rmp_y  # 求临界区后沿中心坐标
    rmp_y = float(rmp_y)
    for line in lines_all:
        if line.touches(Point(rmp_x, rmp_y)):  # 临界区后沿中心坐标在折线段的端点处
            return line
    return 0  # 属于第二、三种情况


def get_rmp_line(cz):  # 获得临界区的后沿中点所在的折线段
    distance = []  # 存储后沿中点到各折线段的距离，为了方便找到最小的距离
    rmp_x = (cz[2].x + cz[3].x) * 0.5
    rmp_y = (cz[2].y + cz[3].y) * 0.5  # 求临界区后沿中心坐标
    cz_rear_midpoint = np.array(Point(rmp_x, rmp_y))  # Point转换为数组类型
    for line in lines:
        Linep2 = np.array(line.coords[1])
        Linep1 = np.array(line.coords[0])
        distance_point_line = point_distance_line(cz_rear_midpoint, Linep1, Linep2)  # 计算点到线段的距离
        distance.append(distance_point_line)
    min_dis = distance[0]
    min_index = 0
    for dis_index in range(len(distance)):  # 遍历点到所有折线段的距离，找到距离最小的那个折线段，就是后沿中点所在的折线段
        if distance[dis_index] < min_dis:  # 判断点是否在折线段上，找寻最小的那个距离
            min_dis = distance[dis_index]
            min_index = dis_index  # 记录最小距离的段的索引
    return lines[min_index]


# 根据车前悬中心数组，去生成临界区序列
def get_cz_from_vfp(c_and_line, veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE):
    for m in range(len(c_and_line)):  # 遍历车前悬中心数组，取生成临界区
        get_cz_frontCenter(c_and_line[m], veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE)
        veh_r = c_and_line[m]
    return


# 求解第一种和第三种情况对应的停止点,line1是后沿中点所在的折线段
def get_sp_oneAndThree(cz, line1, Wv):
    # 先求点到直线的距离,垂线段的距离
    point = [cz[2].x, cz[2].y]  # 点
    vertex_point1 = [line1.coords[0][0], line1.coords[0][1], line1.coords[1][0], line1.coords[1][1]]  # 第一条折线段
    line_point1, line_point2 = np.array(vertex_point1[0:2]), np.array(vertex_point1[2:])  # 转化为数组
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    m = np.linalg.norm(line_point1 - line_point2)
    if m == 0:
        print('error')
        return 0
    else:
        distance = np.abs(np.cross(vec1, vec2)) / m  # 垂线段距离
    # 求两直线相交的交点作为垂足
    x1, x2 = line1.coords[0][0], line1.coords[1][0]
    y1, y2 = line1.coords[0][1], line1.coords[1][1]
    x3, y3 = cz[2].x, cz[2].y
    x = sympy.Symbol('x')
    y = sympy.Symbol('y')
    res = sympy.solve([(y2 - y1) * x + (x1 - x2) * y + x2 * y1 - x1 * y2,
                       (x2 - x1) * x + (y2 - y1) * y - (x2 - x1) * x3 - (y2 - y1) * y3], x, y)  # 折线段和垂线段的交点
    pedal = Point(res[x], res[y])  # 垂足
    if distance > 0.5 * Wv:  # 车头可以继续向前行驶，求停止点
        # 求两条直线line1、临界区的后沿的一半的夹角
        rmp_x = (cz[2].x + cz[3].x) * 0.5
        rmp_y = (cz[2].y + cz[3].y) * 0.5  # 求临界区后沿中心坐标
        line2 = point + [rmp_x, rmp_y]  # 临界区后沿的一半
        angle = GetCrossAngle(vertex_point1, line2)  # 两相邻线段的夹角
        vertex_stop_dis = 0.5 * Wv / tan(radians(angle))  # 拐点距离停车点的距离
        circle_p = Point(rmp_x, rmp_y)
        if vertex_stop_dis < 2e-2:  # 由于存在无限接近的情景，计算机处理小数点后尾数的能力限制，采取这样的近似计算，使临界点直接取临界区后沿中点
            critical_point = circle_p
            stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
            return stop_point
        else:
            circle = circle_p.buffer(vertex_stop_dis).boundary
            critical_point = circle.intersection(
                LineString([circle_p, line1.coords[0]]))  # line3是从临界区后沿到折线段初始点的线段,求出临界点
            stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
            return stop_point
    else:  # 临界点是垂足
        stop_point = get_moved_point(pedal, line1.coords[0], stop_critical_point_distance)
        return stop_point  # 垂足作为临界点


#  获得所在折线段之前的折线段的长度和
def get_last_segments(line):
    last_length = 0
    for i in lines_all:  # 遍历所有车道，找到当前段
        if i != line:
            last_length += i.length
        else:
            return last_length


# 区分情况3和情况2
def disting_type_twoThree(cz):
    line = get_rmp_line(cz)  # 临界区后沿中点所在的折线段
    fmp_x = (cz[0].x + cz[1].x) * 0.5
    fmp_y = (cz[0].y + cz[1].y) * 0.5  # 临界区前沿中心的坐标
    # 前沿中点是否在后沿中点所在的折线段的直线上，求点到直线的距离判断是否为0
    point = [fmp_x, fmp_y]  # 前沿中点
    vertex_point1 = [line.coords[0][0], line.coords[0][1], line.coords[1][0], line.coords[1][1]]  # 第一条折线段
    line_point1, line_point2 = np.array(vertex_point1[0:2]), np.array(vertex_point1[2:])  # 转化为数组
    vec1 = line_point1 - point
    vec2 = line_point2 - point
    m = np.abs(np.cross(vec1, vec2)) / np.linalg.norm(line_point1 - line_point2)  # 前沿中点到后沿中点所在的直线
    if m - 0 < 0.1:  # 不精确解
        print('前沿中点在后沿中点所在的直线上，属于情况2')
        return 2
    else:
        print('前沿中点不在后沿中点所在的直线上，属于情况3')
        return 3


# 情况2和情况4解法的分类
def get_div_two_four(cz, line1, Wv, Lv):  # line1是临界区后沿所在的折线段
    pre_line = LineString([Point(0, 0), Point(1, 1)])
    rmp_x = (cz[2].x + cz[3].x) * 0.5
    rmp_y = (cz[2].y + cz[3].y) * 0.5  # 求临界区后沿中心坐标
    rmp = Point(rmp_x, rmp_y)  # 后沿中点
    x = LineString([Point(line1.coords[0]), rmp]).length  # GE,拐点和临界区后沿中点的距离
    last_dis = LineString([line1.coords[0], rmp]).length  # 所在的折线段的起点到临界区后沿中点的距离
    if abs(last_dis - Lv) < 2e-2 or last_dis - Lv > 0:  # l如果剩余长度大于等于Lv,那么临界点就取临界区后沿中点，
        stop_point = get_moved_point(rmp, line1.coords[0], stop_critical_point_distance)
        return stop_point
    else:  # 剩余长度小于Lv,那么会有两种情况，其中有一种是非常小
        for i in range(len(lines) - 1):
            if i < len(lines) and lines[i + 1] == line1:
                pre_line = lines[i]  # pre_line是临界区后沿所在的折线段的前一个折线段（先经过的）
        pre_line_p = [pre_line.coords[0][0], pre_line.coords[0][1], pre_line.coords[1][0],
                      pre_line.coords[1][1]]
        cur_line_p = [line1.coords[0][0], line1.coords[0][1], line1.coords[1][0], line1.coords[1][1]]  # 当前折线段
        beta = radians(GetCrossAngle(cur_line_p, pre_line_p))  # 前后两条折线段的夹角,转化为弧度
        if x <= 0.5 * Wv * sin(beta):  # 非常小的情况，特别情况（负解）
            stop_point, line = get_sp_four(Lv, Wv, beta, x, line1)  # 返回的是停止点以及它所在的段
            if line.distance(stop_point) < 1e-2:
                print("停止点在临界区后沿中点之前所在的段上")
            else:
                print("不在")
            return stop_point, line
        else:  # 非特别解，情况②
            stop_point, line = get_sp_two(Lv, Wv, beta, x, line1)  # 返回的是停止点以及它所在的段
            if line.distance(stop_point) < 1e-2:
                print("停止点在临界区后沿中点所在的段上")
            else:
                print("不在")
            return stop_point, line


# 情况二的停止点
def get_sp_two(Lv, Wv, beta, x, line1):
    p1 = Point(line1.coords[0])  # 起始端点
    c1 = Lv * Wv * math.sin(beta) * math.sqrt(
        4 * math.pow(Lv, 2) - 4 * Lv * Wv * math.cos(beta) * math.sin(beta) + math.pow(Wv, 2) * math.pow(math.sin(beta),
                                                                                                         2) + 4 * math.pow(
            x, 2) * math.pow(math.cos(beta), 2) - 4 * math.pow(x, 2))
    c2 = 4 * math.pow(Lv, 2) - 4 * math.cos(beta) * Lv * Wv * math.sin(beta) + math.pow(Wv, 2) * math.pow(
        math.sin(beta), 2)
    res2 = (4 * math.pow(Lv, 2) * x - c1 - 2 * Lv * Wv * x * math.cos(beta) * math.sin(beta)) / c2
    res1 = (4 * math.pow(Lv, 2) * x + c1 - 2 * Lv * Wv * x * math.cos(beta) * math.sin(beta)) / c2
    if res2 < x:  # res2是GQ，可以根据G求出停止点
        critical_point = get_moved_point(p1, line1.coords[1], res2)
        stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
        return [stop_point, line1] # 第二种情况下，也要返回临界区后延所在的段，为了和第四种特殊情况一致返回格式
    elif res1 < x:  # res1是GQ，可以根据G求出停止点
        critical_point = get_moved_point(p1, line1.coords[1], res1)
        stop_point = get_moved_point(critical_point, line1.coords[0], stop_critical_point_distance)
        return [stop_point, line1]


# 情况四
def get_sp_four(Lv, Wv, beta, x, line1):
    for i in range(len(lines)):
        if i < len(lines) and lines[i + 1] == line1:
            pre_line = lines[i]  # pre_line是临界区后沿所在的折线段的前一个折线段（先经过的）
    dis = 0.5 * Wv * tan(beta) - x / cos(beta)  # GS是dis,由G求出S坐标
    G = line1.coords[0]  # 起始端点
    circle = G.buffer(dis).boundary
    critical_point = circle.intersection(pre_line)  # 求出临界点
    stop_point = get_moved_point(critical_point, pre_line.coords[0], stop_critical_point_distance)  # 求出停止点
    return [stop_point, pre_line]  # 因为第四种特殊情况下，求停止点行驶距离时，用的是临界区后沿中点所在的段的前一个段，所以需要获得这个段


# 自定义图例形状
def make_legend_arrow(legend, orig_handle,
                      xdescent, ydescent,
                      width, height, fontsize):
    p = mpatches.FancyArrow(0, 0.5 * height, width, 0, length_includes_head=True, head_width=0.75 * height,
                            color='black')
    return p


# 获取临界区的停止点
def get_cz_stop_point():
    for cz_i in CZ:
        line_one = get_type_one(cz_i)  # 获取临界区属于的类型,说明临界区后沿中点在端点处
        if line_one:
            print("临界区的后沿中点在多段线的端点处,属于情况1")
            stop = get_sp_oneAndThree(cz_i, line_one, WVEHICLE)  # 获得临界区后沿中点在折线端点处的情况的停止点
            last_length = get_last_segments(line_one)  # 获得当前段之前的段的长度和
            stop_along_lane = LineString([stop, line_one.coords[0]]).length + last_length  # 停止点距离当前内置车道起点的距离
            stop_dis_from_start.append(stop_along_lane)
            if line_one.distance(stop) < 1e-2:
                print("停止点在临界区后沿中点所在的段上")
            else:
                print("不在")
        else:
            index = disting_type_twoThree(cz_i)  # 区分属于情况二还是三
            line_twTh = get_rmp_line(cz_i)  # 获得临界区后沿中点所在的折线段
            if index == 3:
                stop = get_sp_oneAndThree(cz_i, line_twTh, WVEHICLE)  # 获得情况三的停止点
                last_length = get_last_segments(line_twTh)  # 获得当前段之前的段的长度和
                stop_along_lane = LineString([stop, line_twTh.coords[0]]).length + last_length  # 停止点距离当前内置车道起点的距离
                stop_dis_from_start.append(stop_along_lane)
                if line_twTh.distance(stop) < 1e-2:
                    print("停止点在临界区后沿中点所在的段上")
                else:
                    print("不在")
            elif index == 2:
                stop,line_stop = get_div_two_four(cz_i, line_twTh, WVEHICLE, LVEHICLE)  # 获得情况二或者四的停止点，和停止点所在的段
                last_length = get_last_segments(line_twTh)  # 获得当前段之前的段的长度和
                stop_along_lane = LineString([stop, line_stop.coords[0]]).length + last_length  # 停止点距离当前内置车道起点的距离
                stop_dis_from_start.append(stop_along_lane)
        stop_point_x.append(stop.x)
        stop_point_y.append(stop.y)
        stop_point.append(stop)
        print("No." + str(CZ.index(cz_i) + 1) + "临界区停止点坐标:", stop)
    return

# 绘制临界区及其停止点的图像
def plot_cz_sp_figure():
    # 绘制图像
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
    red_patch = mpatches.Patch(color='black', label='CriticalZone', linewidth='1.2', fill=False)  # 创建矩形图例
    # 设置方向,给内置车道加上箭头
    for k in range(len(internal_point) - 1):
        plt.arrow(internal_point[k].x, internal_point[k].y, internal_point[k + 1].x - internal_point[k].x,
                  internal_point[k + 1].y - internal_point[k].y,
                  head_length=0.6, length_includes_head=True,  # 增加的长度包含箭头部分
                  head_width=0.4, label='Direction', fc='b', ec='b')
    # 设置图例
    arrow = plt.arrow(0, 0, 0.5, 0.6, label='Direction', color='black')
    plt.legend([red_patch, stopp, arrow], ['CriticalZone', 'StopPoint', 'Direction'],
               handler_map={mpatches.FancyArrow: HandlerPatch(patch_func=make_legend_arrow),
                            }, loc='best', fontsize='8', facecolor='orange')
    # 需要设置 plt 的取值范围，为了消除驶入车道和驶出车道的长度过长导致图像过小的问题
    x_min = min(internal_line.xy[0]) - 1.5 * LVEHICLE
    y_min = min(internal_line.xy[1]) - 1.5 * LVEHICLE
    x_max = max(internal_line.xy[0]) + 1.5 * LVEHICLE
    y_max = max(internal_line.xy[1]) + 1.5 * LVEHICLE
    plt.xlim((x_min, x_max))
    plt.ylim((y_min, y_max))
    ax.set_aspect(1)  # 使得xy轴的单位长度相等
    plt.xticks(size=15)  # 设置xy轴文字的大小
    plt.yticks(size=15)
    plt.title("Conflict zone demo", fontsize=25)
    plt.xlabel("X", fontsize=20)
    plt.ylabel("Y", fontsize=20)
    plt.show()
    return


if __name__ == '__main__':
    # ---解析路网文件，获取shape值中的点坐标
    # net_xml = Net("C:/Users/86188/Desktop/example/single_lane.net.xml")
    net_xml = Net("D:/CyberCarSimulation_SUMO/SumoCurveLane/sumo_net_curve/merge614.net.xml")
    con_matrix = net_xml.conflict_matrix()  # 解析路网，得到路网的冲突矩阵
    print("路网的connection冲突矩阵：\n", con_matrix)
    from_lane = input('FromLane:')  # 定义驶入边
    to_lane = input('ToLane:')  # 定义驶出边
    changed_laneid = net_xml.change_laneid(from_lane, to_lane)  # 转换id形式
    shape = net_xml.get_shape(changed_laneid, from_lane, to_lane)  # 返回的是一个内置车道的shape字符串列表，与xml中保持一致
    POINT_NUM = len(shape)  # 端点数目，左转右转是7个点，直行是4个点
    POLYLINE_NUM = POINT_NUM - 1  # 折线段数目
    point_all = [0] * POINT_NUM  # 存储的是出入车道以及内置车道的点集
    poly_length = [0] * POLYLINE_NUM  # 存储每段折线段长度的列表
    for string in shape:
        s2 = string.split(',')
        point_all[count] = Point(float(s2[0]), float(s2[1]))
        count += 1
        if count == len(shape):
            break
    for i in point_all[1::]:  # 只包含内置车道和输出车道的端点
        point.append(i)
    for i in point_all[1:len(point_all) - 1:1]:
        internal_point.append(i)
    internal_line = get_polylines(internal_point)  # 内置车道段
    polyline_all = get_polylines(point_all)  # 所有段线的总长度
    polyline = get_polylines(point)  # 内置车道和驶出车道的多段线长度
    for i in range(len(point) - 1):
        l = LineString([point[i], point[i + 1]])
        lines.append(l)  # lines仅包含内置车道折线段
    for i in range(len(point_all) - 1):  # lines_all是全部的折线段
        l = LineString([point_all[i], point_all[i + 1]])
        lines_all.append(l)
    get_polyline_length(point)
    get_polyi(poly_length)  # 获得从内置车道第一个端点到后几个端点的值
    poly_i = [0] + poly_i  # poly_i得到折线段每个点到初始点的距离
    if len(internal_point) == 2:  # 直行的临界区划分的内置车道端点为2
        # 如果是直行，调用这此函数计算车头,并计算临界区
        get_straight_vehicle_fclist(STRAIGHT_DSAFE)  # 对于直行，车长加安全距离作为新车长，所以车头是加过安全距离后的
        veh_r = point[0]  # 初始化
        get_cz_from_vfp(c_and_line, veh_r, 0, WSAFE, WVEHICLE, LVEHICLE + STRAIGHT_DSAFE)  # 直行的dsafe=0,且车长是加过安全距离的
        print("直行车辆的前悬中心坐标序列")
        get_straight_vhe_fc()
        for i in straight_vhe_fc:
            print(i)
    else:
        # 如果是转弯，调用此函数计算车头
        get_turning_vehicle_fclist()
        veh_r = point[0]  # 初始化
        get_cz_from_vfp(c_and_line, veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE)  # 获得无安全距离时的临界区划分
        DSAFE = get_new_dsafe(CZ)  # 获取消除空白间隙的最小的安全距离
        # DSAFE=0
        print("安全距离为：", DSAFE)
        CZ = []  # 更新临界区序列
        get_cz_from_vfp(c_and_line, veh_r, DSAFE, WSAFE, WVEHICLE, LVEHICLE)  # 获得无安全距离时的临界区划分
        print("转弯车辆前悬中心坐标序列：")
        for p_vf in c_and_line:  # 输出车辆前悬中心
            print(p_vf)
            print("误差：", polyline.distance(p_vf))  # 检测车头是不是在折线段上,有误差
    along_lane = [0] + along_lane  # 第一个临界区后沿中点就是起始点，为0
    print("\n")
    for a_dis in range(len(along_lane)):
        print("第" + str(a_dis + 1) + "个临界区后沿沿着内置车道行驶的距离：", along_lane[a_dis])
    print(along_lane)
    for n in range(len(poly_i)):
        print("第" + str(n + 1) + "个端点到内置车道初始点的行驶距离为：", poly_i[n])
    cz_index = get_cz_rm()
    for p_vr in cz_index:  # 输出临界区序列，后沿中心坐标
        print("第" + str(cz_index.index(p_vr) + 1) + "个临界区后沿中心位置：", p_vr)
        print("误差", polyline.distance(p_vr))  # 检测车尾是不是在折线段上,有误差
    for index in range(len(CZ)):
        print("第" + str(index + 1) + "个临界区的顶点的坐标：")
        for cz_pos in CZ[index]:
            print(cz_pos)
    #get_cz_stop_point()  # 获得临界区停止点
    for i in range(1, len(stop_dis_from_start), 1):
        stop_dis_from_start[i] -= lines_all[0].length  # 计算沿着输入车道初始点行驶的距离，但是在交叉路口中，是按照内置车道起点开始的，故从进入内置车道起，将距离都减去起始车道的距离
    for i in stop_dis_from_start:
        print("第" + str(stop_dis_from_start.index(i) + 1) + "个临界区停止点沿着所在车道行驶的距离为", i)

    plot_cz_sp_figure()  # 绘图
