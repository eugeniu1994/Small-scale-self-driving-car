#!/usr/bin/env python
import rospy
from math import *
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Point, Twist
import numpy as np
import math
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import bisect
import matplotlib.pyplot as plt
import math
import numpy as np
import time as  timp

#this file is used to follow some path

#this class defines the spline function with x and y arguments
class Spline:
    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []
        self.x = x
        self.y = y

        self.nx = len(x)
        h = np.diff(x)

        self.a = [iy for iy in y]
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)

        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                 (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
                 self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0
        return result

    def calcd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None
        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                       h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B

#this class used Spline defined above to ompute 2D spline
#is used for path planning
class Spline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s
    def calc_position(self, s):
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

 #used to create pose message
def createPose(px, py, pz, ox, oy, oz, ow):
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = '/map'
    pose = PoseStamped()
    pose.header = h
    pose.pose.position.x = float(px)
    pose.pose.position.y = float(py)
    pose.pose.position.z = float(pz)
    pose.pose.orientation.x = float(ox)
    pose.pose.orientation.y = float(oy)
    pose.pose.orientation.z = float(oz)
    pose.pose.orientation.w = float(ow)

    return pose

#this function compute spline function with ds:= distance between points
def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))
    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

k = 0.5 #used to compute curvature for spline functions
Kp = 1.0  # position gain for PD controller
dt = 0.1  # [s] time difference
L = 0.3  # [m] Wheel base of the car
max_steer = np.radians(45.0)  # [rad] max steering angle

path = np.empty(0)
pose = np.empty(3, dtype=float)  # x, y, angle in rad
velocity = np.empty(6, dtype=float)  # linear 3D, angular 3D
perpVector = Pose()

# this class defines the position of the car, x,y,angle and velocity
class State(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

#define euclidean distance
def euclid(x1, x2, y1, y2):
    return np.sqrt(pow(x1 - x2, 2) + (pow(y1 - y2, 2)))

#compute the error, (here velocity error needs to be added, and integral part also)
def pid_control(target, current):
    return Kp * (target - current)

#this function is used to compute the angular velocity for Twist messages
def stanley_control(state, cx, cy, cyaw, last_target_idx):
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)
    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    theta_d = np.arctan2(k * error_front_axle, state.v)
    delta = theta_e + theta_d

    return delta, current_target_idx

 # used to normalize angle diff
def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

#used to get angle from quaternion
def twoD_quat_to_radian(oz, ow):
    return normalize_angle(2 * atan2(oz, ow))

#used to get the next state from desired path
def calc_target_index(state, cx, cy):
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

#get data from odometry, position x,y, and compute angle from quaternion
def callback_odom(data):
    pose[0] = data.pose.position.x
    pose[1] = data.pose.position.y
    oz = data.pose.orientation.z
    ow = data.pose.orientation.w
    pose[2] = twoD_quat_to_radian(oz, ow)

    #rospy.loginfo("x:{}, y:{}, w:{}".format(round(pose[0], 2), round(pose[1], 2),round(pose[2],2)))

def main():
    rospy.init_node("camera_values")
    rospy.Subscriber("/slam_out_pose", PoseStamped, callback_odom)  #subscribe to EKF localization

    pub_speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    #some global via points
    ax = [0, 3, 5, 5.5, 5.7, 6, 18, 19, 19, 17, 10]
    ay = [0, 0, 0, -0.2, -4.3, -8, -8.8, -8.6, 0.1, 0.4, 0.4]

    global path
    global pose
    global k
    global velocity
    global perpVector
    # compute spline function for x and y position
    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=0.1)

    target_speed = 4.0 / 3.6  # [m/s]

    state = State(x=pose[0], y=pose[1], yaw=np.radians(0.0), v=0.0)

    last_idx = len(cx) - 10
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    # get desired position
    target_idx, _ = calc_target_index(state, cx, cy)

    twistmsg = Twist()
    rate = rospy.Rate(1)  # 10hz
    # compute trajectory
    while not rospy.is_shutdown() and last_idx > target_idx:
        if target_idx >= last_idx:
            twistmsg.linear.x = 0.0
            twistmsg.angular.z = 0.0
            pub_speed.publish(twistmsg)
            rospy.loginfo("destination reatched last_idx {} , target_idx {}".format(last_idx,target_idx))
        else:
            ai = pid_control(target_speed, state.v)

            state.x = pose[0]
            state.y = pose[1]
            state.yaw = pose[2]
            #compute angular velocity
            di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
            state.update(ai, di)
            time += dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)

            #create Twist message for cmd_vel topic
            twistmsg.linear.x = state.v
            twistmsg.linear.y = 0.
            twistmsg.linear.z = 0.
            twistmsg.angular.x = 0.
            twistmsg.angular.y = 0.
            twistmsg.angular.z = -di
            pub_speed.publish(twistmsg)

            #rospy.loginfo("v:{}, w:{}, x:{}, y:{} l_idx {} , t_idx {}".format(round(state.v, 1), round(di, 1), round(pose[0],1),round(pose[1],1),last_idx, target_idx))

        pub_speed.publish

    twistmsg.linear.x = 0.0
    twistmsg.angular.z = 0.0
    pub_speed.publish(twistmsg)
    rospy.loginfo("destination reatched last_idx {} , target_idx {}".format(last_idx, target_idx))
    pub_speed.publish

    assert last_idx >= target_idx, "Cannot reach goal"
    rate.sleep()

if __name__ == '__main__':
    main()
