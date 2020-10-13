#!/usr/bin/env python

import sys
import time as timp
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import CompressedImage
import message_filters
from cv_bridge import CvBridge
import os, rospkg
from std_msgs.msg import String
#from scipy.spatial import distance as dist

from collections import OrderedDict
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
import bisect
from geometry_msgs.msg import Pose, Twist
from camera_values.msg import MyTemplate
from sensor_msgs.msg import LaserScan
import torch

from torch.utils.data import DataLoader
from torch import Tensor
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data.dataset import Dataset
from torch.autograd import Variable

usePcCamera = False  # true if want to use real car
useCpu = True # used to run traffic sign recognition on CPU or GPU
#get param use_CPU, if true use CPU, else, use GPU, default is CPU
useCpu = rospy.get_param('~ARG1', True)
rospy.loginfo("run on {}".format("CPU" if useCpu == True else "GPU"))

if usePcCamera:
    slopeThreshold = 0.4   # used to filter traffic lane, real car
    minTrafficSignRadius = 20 #start tracking traffic sign from this radius
    safetyThTraffic = 30 #extra pixels when take detected region from frame
    centerOfTheFrame = 320  # middle points of the frame, since frame width = 640
else:
    slopeThreshold = 0.3  # used to filter traffic lane for simulation
    minTrafficSignRadius = 5 #start tracking traffic sign from this radius
    safetyThTraffic = 5 #extra pixels when take detected region from frame,
    centerOfTheFrame = 64  # for simulation

#globa device, is used, in case to test traffic sign detection on GPU or CPU
device = torch.device("cuda")
pose = np.empty(3, dtype=float)  # x, y, angle in rad
global ax #global x path
global ay  #global y path
ax = []
ay = []
k = 0.5 #used to compute curvature for spline functions
Kp = 1.0  # position gain for PD controller
dt = 0.1  # [s] time difference
L = 0.3  # [m] Wheel base of the car
max_steer = np.radians(45.0)  # [rad] max steering angle

rospack = rospkg.RosPack()

def createPose(px, py, pz, ox, oy, oz, ow): #used to create pose message
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

def normalize_angle(angle): # used to normalize angle diff
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def twoD_quat_to_radian(oz, ow): #used to get angle from quaternion
    return normalize_angle(2 * math.atan2(oz, ow))

def check(label): # this function return label of y_hat, used for traffic sign recognition
    if label == 0:
        return 'Nothing'
    elif label == 1:
        return 'Stop'
    elif label == 2:
        return 'Right'
    elif label == 3:
        return 'Left'
    elif label == 4:
        return 'Right'
    elif label == 5:
        return 'Left'
    elif label == 6:
        return 'Nothing'
    return ''

def segmentForCircles(frame):  #used to select specific region for traffic sign detection
    height = frame.shape[0]
    width = frame.shape[1]

    th = int(height / 2.5)
    polygons = np.array([
        [(0, height), (width, height), (width, th), (0, th)]
    ])
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, polygons, 255)
    segment = cv2.bitwise_and(frame, mask)

    # cv2.imshow("segment circle " , segment)
    return segment

def TrafficSign(frame, classifier):
    if usePcCamera:
        # gray image and apply blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0);
        img = cv2.medianBlur(gray, 5)
        img = segmentForCircles(img) #get section for traffic sign detection

        # apply cv circle detection, this parameters are tuned for 4WD car
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 500, param1=100, param2=50, minRadius=20,
                                   maxRadius=200)  # for real car
    else:
        # gray image and apply blur
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img = cv2.medianBlur(gray, 5)
        rows = img.shape[0]

        # apply cv circle detection, this parameters wre tuned for simulation
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, rows / 8, param1=100, param2=30, minRadius=1,
                                   maxRadius=50)  # for simulation
    detectedClass = ''
    if not circles is None:
        circles = np.around(circles)
        max_r, max_i = 0, 0
        # here we iterate over all detected circles, and keep the bigger circle
        for i in range(len(circles[:, :, 2][0])):
            if int(circles[:, :, 2][0][i]) > 50 and int(circles[:, :, 2][0][i]) > max_r:
                max_i = i
                max_r = int(circles[:, :, 2][0][i])
        x1, y, r = circles[:, :, :][0][max_i]
        x1, y, r = int(x1), int(y), int(r)

        cv2.circle(frame, (x1, y), r, (0, 255, 0), 2)
        cv2.circle(frame, (x1, y), 2, (0, 0, 255), 3)
        # check if we can start track it.
        if y > r and x1 > r and r > minTrafficSignRadius:
            r = r + safetyThTraffic
            # get detected region from image
            square = frame[y - r:y + r, x1 - r:x1 + r]
            if square.any():
                img = cv2.resize(square, (IMG_SIZE, IMG_SIZE)) / 255.0
                # cv2.imshow('square', img)
                test_x = Tensor(img).view(-1, 3, IMG_SIZE, IMG_SIZE).float()
                if useCpu:
                    pred = classifier.forward(test_x)
                    y_hat = np.argmax(pred.data) #get the index from max value
                else:
                    test_x = test_x.to(device) #used GPU, convert to CUDA
                    pred = classifier.forward(test_x)
                    y_hat = pred.cpu().data.numpy().argmax() #get the index from max value

                detectedClass = check(y_hat)
                # rospy.loginfo("detected class {}....y_hat {}...............".format(detectedClass,y_hat))

    return frame, detectedClass, circles

def rgba2rgb(rgba, background=(255, 255, 255)):  #this function was used to convert the frames from rgba to rgb
    row, col, ch = rgba.shape

    if ch == 3:
        return rgba

    assert ch == 4, 'RGBA image has 4 channels.'

    rgb = np.zeros((row, col, 3), dtype='float32')
    r, g, b, a = rgba[:, :, 0], rgba[:, :, 1], rgba[:, :, 2], rgba[:, :, 3]

    a = np.asarray(a, dtype='float32') / 255.0

    R, G, B = background

    rgb[:, :, 0] = r * a + (1.0 - a) * R
    rgb[:, :, 1] = g * a + (1.0 - a) * G
    rgb[:, :, 2] = b * a + (1.0 - a) * B

    return np.asarray(rgb, dtype='uint8')

def canny_filter(frame): # this function is used for traffic lane detection, to gray the image and apply blur
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny

def segmentation(frame, img): #used for lane detection, to get the specific region for lane searching
    height = frame.shape[0]
    width = frame.shape[1]

    if usePcCamera:
        th = 200
        # polygon is the region where to search
        polygons = np.array([
            [(0, height), (width, height), (int(width - th), int(height / 1.5)), (int(th), int(height / 1.5))]
        ])
        # cv2.imshow("segmentation", img)
        #cv2.line(img, (0, height), ((int(th), int(height / 1.5))), (255, 0, 0), 2)
        #cv2.line(img, (width, height), (int(width - th), int(height / 1.5)), (255, 0, 0), 2)
        #cv2.line(img, (int(th), int(height / 1.5)), (int(width - th), int(height / 1.5)), (255, 0, 0), 2)
    else:  # for simulation
        # region where to search in simulation
        polygons = np.array([
            [(0, height), (width, height), (width, height - 20), (int(width / 2), 50), (0, height - 20)]
        ])

    # apply mask and bitwise operation to get desired segment
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, polygons, 255)
    segment = cv2.bitwise_and(frame, mask)

    return segment

def calculate_coordinates(frame, parameters): # used to compute image coordinates for lane detection
    slope, intercept = parameters
    y1 = frame.shape[0]
    if usePcCamera:
        y2 = int(y1 / 1.5)  #tuned for real 4WD car
    else:
        y2 = 80   #tuned for simulation

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

def calculate_lines(frame, lines):
    left = []
    right = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # returns  slope and y-intercept
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            if parameters is not None:
                slope = parameters[0]
                # check the slope to filter, undesired lines
                if np.abs(slope) > slopeThreshold:
                    y_intercept = parameters[1]
                    # If slope is negative, the line is to the left of the lane, and otherwise, the line is to the right of the lane
                    if slope < 0:
                        left.append((slope, y_intercept))
                    else:
                        right.append((slope, y_intercept))

                    #cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 5)

        # Averages out all the values for left and right into a single slope and y-intercept value for each line
        left_avg = np.average(left, axis=0)
        right_avg = np.average(right, axis=0)

        # Calculates the x1, y1, x2, y2 coordinates for the left and right lines
        if left_avg is not None and left_avg.any() and right_avg is not None and right_avg.any() and len(left) > 0 and len(right) > 0:
            left_line = calculate_coordinates(frame, left_avg)
            right_line = calculate_coordinates(frame, right_avg)
            return np.array([left_line, right_line]), 0
        elif left_avg is not None and left_avg.any() and len(left) > 0:
            left_line = calculate_coordinates(frame, left_avg)
            return np.array([left_line, left_line]), -1
        elif right_avg is not None and len(right) > 0:
            right_line = calculate_coordinates(frame, right_avg)
            return np.array([right_line, right_line]), 1
        else:
            return (None,0)

def visualize_lines(frame, lines):  #used to visualize the result for lane detection
    lines_visualize = np.zeros_like(frame)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 4)
    return lines_visualize

def most_frequent(List):  #used for traffic sign tracking, to get the most frequent detected class from a buffer
    return max(set(List), key=List.count)

class State(object): #this class defines the position of the car, x,y,angle and velocity
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

#used to get the next state from desired path
def calc_target_index(state, cx, cy):
    # front angle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

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

def getBiggestCircle(circles): #used to filter all circles, and get the bigger one
    rv = None
    radius = None
    if not circles is None:
        circles = np.around(circles)
        max_r, max_i = 0, 0
        for i in range(len(circles[:, :, 2][0])):
            if int(circles[:, :, 2][0][i]) > 50 and int(circles[:, :, 2][0][i]) > max_r:
                max_i = i
                max_r = int(circles[:, :, 2][0][i])
        x1, y, r = circles[:, :, :][0][max_i]
        x1, y, r = int(x1), int(y), int(r)
        if y > r and x1 > r:
            box = []
            (w, h) = (2 * r, 2 * r)
            box.append(x1 - r)
            box.append(y - r)
            box.append(w)
            box.append(h)

            box = np.array(box)
            rv = box
            radius = r

    return rv, radius

#euclidean distance, used to compute the distance between the car position and desired position
def euclidean_distance(x1, y1, x2, y2):
    return np.sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))

NUM_CLASSES = 7 #number of detected classes, used for CNN params
IMG_SIZE = 32 # image size, input for CNN

class ConvNet(nn.Module): # this class define a convolutional layer, with max pooling and batch normalization

    def __init__(self, in_c, out_c,
                 kernel_size,
                 padding_size='same',
                 pool_stride=2,
                 batch_norm=True):
        super(ConvNet, self).__init__()

        if padding_size == 'same':
            padding_size = kernel_size // 2
        self.conv = nn.Conv2d(in_c, out_c, kernel_size, padding=padding_size)
        self.max_pool2d = nn.MaxPool2d(pool_stride, stride=pool_stride)
        self.batch_norm = batch_norm
        self.batch_norm_2d = nn.BatchNorm2d(out_c)

    def forward(self, x):
        x = self.max_pool2d(nn.functional.leaky_relu(self.conv(x)))

        if self.batch_norm:
            return self.batch_norm_2d(x)
        else:
            return x

#this function is used to compute the output of CNN,
#in order to coonect the CNN with fully connected layers
#it serves as a flatten layer
def get_convnet_output_size(network, input_size=IMG_SIZE):
    input_size = input_size or IMG_SIZE

    if type(network) != list:
        network = [network]

    in_channels = network[0].conv.in_channels

    output = Variable(torch.ones(1, in_channels, input_size, input_size))
    output.require_grad = False
    for conv in network:
        output = conv.forward(output)

    return np.asscalar(np.prod(output.data.shape)), output.data.size()[2]

#this class defines the neural network architecture
#3 CNN layer, 2 fully connected, softmax operator
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = ConvNet(3, 150, kernel_size=5, padding_size=0)
        self.conv2 = ConvNet(150, 200, kernel_size=3, padding_size=0)
        self.conv3 = ConvNet(200, 300, kernel_size=3, padding_size=0)

        inp, _ = get_convnet_output_size([self.conv1,
                                          self.conv2,
                                          self.conv3],
                                         IMG_SIZE)
        self.fc1 = nn.Linear(inp, 50)
        self.fc2 = nn.Linear(50, NUM_CLASSES)

    def forward(self, x):
        x = self.conv3(self.conv2(self.conv1(x)))
        x = x.view(x.size(0), -1)

        x = F.relu(self.fc1(x))
        x = F.dropout(x, training=self.training)
        x = self.fc2(x)
        return F.log_softmax(x)

#this is the main class, defines subscribers and publisher
class camera_data:
    def __init__(self):
        self.image = None
        self.bridge = CvBridge() #used to get image from real car
        self.pub = rospy.Publisher('/global_path', Path, queue_size=10) #used to vizualize data on rviz
        if usePcCamera:
            self.image_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.callback) #image topic for 4WD car
        else:
            self.image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)  #image topic for simulation

        self.pub_speed = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path = os.path.join(rospack.get_path("camera_values"), "launch", "Traffic_GTSRB.pt")

        self.classifier = Net()
        if useCpu == False:  # if not using CPU, conver the classifier to CUDA
            self.classifier = self.classifier.to(device)
        self.classifier.load_state_dict(torch.load(self.path))
        self.classifier.eval()

        self.radiusThreshold = 30 #used to take decision, in traffic sign tracking
        self.minRadiusThreshold = 10 #start tracking from this radius
        self.detectedTrafficSigns = []
        self.TookDecision = False
        self.cls = None
        self.midle = None
        self.turningThreshold = 5 # used to start turning, when follow the traffic lane
        # used for communication with motion tracking node
        self.semaphore = rospy.Publisher('/semaphore', MyTemplate, queue_size=1)
        self.semaphore_subscriber = rospy.Subscriber("/semaphore", MyTemplate, self.callbackSemaphore)
        self.AddGlobalPoints = False
        self.GlobalX = []
        self.GlobalY = []
        self.switchToGlobal = False

        # used to avoid object collision
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, self.laserCallback, queue_size=1)
        self.obstacle_found = False
        self.OBSTACLE_THRESHOLD = 1.0 # distance to obstacle

        # publish frames to /imagePublisher topic, to visualize in rviz the perception result
        self.imagePublisher = rospy.Publisher("/imagePublisher", Image)

    def laserCallback(self, laser_data):
        distance_front = laser_data.ranges[170:190] #get lidar ranges in front of the car
        # check them with threshold, if too close, set flag to stop the car
        for d in distance_front:
            if d < self.OBSTACLE_THRESHOLD:
                self.obstacle_found = True # obstacle in front
                break
            else:
                self.obstacle_found = False #no obstacle

    def TrafficLane(self, frame):  #this function perform traffic lane detection
        canny = canny_filter(frame) #apply edge detection
        # cv2.imshow("canny", canny)
        segment = segmentation(canny, frame)  #extract desired region from image
        #cv2.imshow("segment", segment)

        if usePcCamera:
            # cv lines detection , tuned for real 4WD car
            hough = cv2.HoughLinesP(segment, rho=1, theta=np.pi / 360, threshold=10, lines=np.array([]),
                                    minLineLength=70, maxLineGap=20)  # real world
        else:
            # cv lines detection, tuned for simulation
            hough = cv2.HoughLinesP(segment, rho=3, theta=np.pi / 180, threshold=50, lines=np.array([]),
                                    minLineLength=5, maxLineGap=20)  # simulation

        # compute traffic lanes from detected lines
        try:
            lines, direction = calculate_lines(frame, hough)
        except:
            lines = None
            direction = 0
        self.publishPoints(lines, direction)  #check the midle point

        lines_visualize = visualize_lines(frame, lines)  # visualize lanes
        if lines_visualize is not None: #visualize traffic lanes and publish result to some imagePublisher topic, to visualize in rviz
            # cv2.imshow("hough", lines_visualize)
            output = cv2.addWeighted(frame, 0.9, lines_visualize, 1, 1)
            output_msg = CvBridge().cv2_to_imgmsg(output)  # output_msg = CvBridge().cv2_to_imgmsg(output, )
            output_msg.header.stamp = rospy.get_rostime()

            cv2.imshow("output", output)

            self.imagePublisher.publish(output_msg)

    def publishPoints(self, lanes, direction):  #check if traffic lanes are detected, and also if there is only one late detected, instead of both
        if lanes is not None:
            s = 0
            if usePcCamera:
                # rospy.loginfo("dir is {}".format(direction))
                for x1, y1, x2, y2 in lanes:
                    s = s + x2
                s = s / 2
                self.midle = s
                if direction == -1:  # is left
                    self.midle = self.midle + 30
                elif direction == 1:  # is right
                    self.midle = self.midle - 30
            else:
                for x1, y1, x2, y2 in lanes:
                    s = s + x2
                s = s / 2
                self.midle = s
                if direction == -1:  # is left
                    self.midle = self.midle + 10
                elif direction == 1:  # is right
                    self.midle = self.midle - 10
        else:
            self.midle = None

    def callbackSemaphore(self, data):  #used for communication with motion controller
        type = data.type
        #rospy.loginfo("Type from Semaphore is {}".format(type))
        if type == "follow":
            self.TookDecision = False
            self.cls = None

    def callback(self, ros_data):  #used to get frames from camera
        original = self.bridge.imgmsg_to_cv2(ros_data, 'bgr8') # use bridge for frames
        original = rgba2rgb(original)  #convert from rgba to rgb
        # apply traffic sign detection & recognition
        original, detectedClass, circles = TrafficSign(original, self.classifier)
        self.TrafficLane(original) # apply traffic lane detection

        self.image = original
        cv2.imshow("Cam data", self.image)
        cv2.waitKey(2)

        if self.obstacle_found: # check if there is no obstacle in front of the car
            rospy.loginfo("------Obstacle in th front---------")
            customMessage = MyTemplate()
            customMessage.ObstacleDetected = "yes"
            self.semaphore.publish(customMessage)

        else:
            if self.switchToGlobal == False:
                bbox, radius = getBiggestCircle(circles)
                # get detected traffic sign and start tracking
                if radius is not None:
                    if radius >= self.minRadiusThreshold: # used to start tracking
                        if detectedClass is not None and detectedClass is not "": self.detectedTrafficSigns.append(
                            detectedClass)
                    if radius >= self.radiusThreshold: #erase tracking buffer here and take decision
                        self.cls = most_frequent(self.detectedTrafficSigns)
                        rospy.loginfo("Radius: {}, Class:{}".format(round(radius, 2), self.cls))
                        self.detectedTrafficSigns = []

                if self.TookDecision == False:
                    ax = []
                    ay = []
                    distance0 = 0.5  #distance in front for the first local via point
                    xi_ = pose[0] + distance0 * np.cos(pose[2])
                    yi_ = pose[1] + distance0 * np.sin(pose[2])
                    ax.append(round(pose[0], 2))
                    ay.append(round(pose[1], 2))
                    ax.append(round(xi_, 2))
                    ay.append(round(yi_, 2))

                    distanceX = 0.5 #step for second local via point
                    turn = np.radians(0) # no turn initially
                    if self.midle is not None:  # here check the traffic lanes,  64 is th center x of the image
                        diff = self.midle - centerOfTheFrame
                        desired_angle = 0
                        if np.abs(diff) > self.turningThreshold:
                            desired_angle = 23
                            turn = np.radians(desired_angle) if diff < 0 else np.radians(-desired_angle)
                        else:
                            turn = np.radians(desired_angle)

                    if self.cls is not None and self.cls is not "" and self.cls is not "-":
                        if self.cls == "Right":
                            distanceX = 2.0
                            turn = np.radians(-75)
                            if self.AddGlobalPoints == True:
                                turn = np.radians(75)
                            self.AddGlobalPoints = True
                        elif self.cls == "Left":
                            distanceX = 2.0
                            turn = np.radians(75)
                        elif self.cls == "Stop":
                            self.switchToGlobal = False# True
                            rospy.loginfo("---------------------------------------------------------------------")
                    # compute the last local via point, based on traffic sign and traffic lane
                    rv = pose[2] + turn
                    xi_2 = xi_ + distanceX * np.cos(rv)
                    yi_2 = yi_ + distanceX * np.sin(rv)
                    ax.append(round(xi_2, 2))
                    ay.append(round(yi_2, 2))

                    #compute spline function for x and y position
                    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=0.2)
                    target_speed = 4.5 / 3.6  # [m/s] linear velocity
                    max_simulation_time = 1000.0
                    state = State(x=pose[0], y=pose[1], yaw=pose[2], v=0.0)
                    last_idx = len(cx) - 1
                    time = 0.0
                    x = [state.x]
                    y = [state.y]
                    yaw = [state.yaw]
                    v = [state.v]
                    t = [0.0]
                    #get desired position
                    target_idx, _ = calc_target_index(state, cx, cy)
                    #compute trajectory
                    while max_simulation_time >= time and last_idx > target_idx:
                        ai = pid_control(target_speed, state.v)
                        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
                        state.update(ai, di)
                        time += dt

                        x.append(state.x)
                        y.append(state.y)
                        yaw.append(state.yaw)
                        v.append(state.v)
                        t.append(time)
                    #compute path
                    myPath = Path()
                    if not len(x) == 0:
                        for i in range(0, len(x)):
                            xi = x[i]
                            yi = y[i]
                            zi = 0.0
                            quaternion = quaternion_from_euler(0, 0, yaw[i])
                            ox = quaternion[0]
                            oy = quaternion[1]
                            oz = quaternion[2]
                            ow = quaternion[3]
                            myPath.poses.append(createPose(xi, yi, zi, ox, oy, oz, ow))

                    assert last_idx >= target_idx, "Cannot reach goal"

                    path = myPath
                    h = Header()
                    h.stamp = rospy.Time.now()
                    h.frame_id = '/map'
                    path.header = h

                    #publish desired path
                    self.pub.publish(path)
                    self.TookDecision = True

                    # compute custom msg, and send it to motion controller, to follow local path
                    customMessage = MyTemplate()
                    customMessage.xtrajectory = ax
                    customMessage.ytrajectory = ay
                    customMessage.type = "plan"
                    #rospy.loginfo("ax:{},  ay:{}".format(ax, ay))

                    #save global via points to switch to global planner
                    if self.AddGlobalPoints:
                        #rospy.loginfo("Gx:{},  Gy:{}".format(self.GlobalX, self.GlobalY))
                        self.GlobalX.append(round(pose[0], 2))
                        self.GlobalY.append(round(pose[1], 2))
                        customMessage.useGlobalPath = "no"
                    self.semaphore.publish(customMessage)
            else:
                #switched to global planner
                customMessage = MyTemplate()
                customMessage.GlobalX = self.GlobalX
                customMessage.GlobalY = self.GlobalY
                customMessage.useGlobalPath = "yes"
                self.semaphore.publish(customMessage)

#get data from odometry, position x,y, and compute angle from quaternion
def callback_odom(data):
    pose[0] = data.pose.position.x
    pose[1] = data.pose.position.y

    oz = data.pose.orientation.z
    ow = data.pose.orientation.w
    pose[2] = twoD_quat_to_radian(oz, ow)


def run(args):
    camera = camera_data()
    rospy.Subscriber("/slam_out_pose", PoseStamped, callback_odom)  #subscribe to EKF localization result
    rospy.loginfo("camera_values started simulation mode")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('stopping camera_values publish path')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_values', anonymous=True) #start the node
    # get param use_CPU, if true use CPU, else, use GPU, default is CPU
    useCpu = rospy.get_param('~ARG1', True)
    rospy.loginfo("run on {}".format("CPU" if useCpu == True else "GPU"))
    run(sys.argv)
