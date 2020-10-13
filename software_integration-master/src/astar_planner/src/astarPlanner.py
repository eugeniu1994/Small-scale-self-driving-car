#!/usr/bin/env python

"""
This file was created to generate a global path using astar.
"""

import rospy
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2
from threading import Thread
from heapq import *
from astar_planner.srv import getAstarPath,getAstarPathResponse
import cv2
from scipy import ndimage
from scipy.misc import imsave


pose = None


class GetMap(object):
    """ Subscribes to hector SLAM's position and odometry to compute a path for the robot to drive. """

    def __init__(self):
        """ Sets subscribers to the map and position up. """
        self.n = rospy.init_node("astar_planner", anonymous=True)
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_subscriber_callback)
        self.odom_subscriber = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)
        self.grid_map = []
        """astar is made as a service"""
        self.srv = rospy.Service('/get_astar_path', getAstarPath, self.srvCallback)
        rospy.loginfo("service is running")
        self.path_publisher = rospy.Publisher("/astar_path", Path, queue_size=10)
        rospy.spin()

    def callback_odom(self, data):
        global pose
        pose = [0., 0.]
        pose[0] = data.pose.pose.position.x
        pose[1] = data.pose.pose.position.y

    def map_subscriber_callback(self, map_msg):
        global map
        map = map_msg
        #rospy.loginfo("received msg")

    def srvCallback(self, request):
        """ Get the goal position from the service request"""
        global goal
        goal = [0., 0.]
        goal[0] = request.goal.pose.position.x
        goal[1] = request.goal.pose.position.y

        lm_info = map.info
        lm_data = map.data
        width = lm_info.width
        height = lm_info.height

        """reshape the array as a grid"""
        lm = np.array(lm_data).astype(np.uint8).reshape((height, width))
        lm[lm == -1] = 1
        #lm[lm == 100] = 1
        self.grid_map= lm
        self.grid_map = np.flip(self.grid_map,0)
        self.grid_map = np.rot90(self.grid_map,3)

        """inflating the obstacles"""
        struct2 = ndimage.generate_binary_structure(2, 2)
        self.grid_map = ndimage.binary_dilation(self.grid_map, structure=struct2, iterations=5).astype(self.grid_map.dtype)
        

        """get the world co-ordinates from the grid"""
        vector_start = [int((pose[0] -lm_info.origin.position.x) / lm_info.resolution), int((pose[1]-lm_info.origin.position.y) / lm_info.resolution)]
        vector_end = [int((goal[0]-lm_info.origin.position.x) / lm_info.resolution), int((goal[1]-lm_info.origin.position.y) / lm_info.resolution)]

        """Calculating the shortest path"""
        output_path = astar(self.grid_map, (vector_start[0],vector_start[1]),(vector_end[0],vector_end[1]))

        # Check whether path has been found
        if output_path == True or output_path == False:
            return getAstarPathResponse(False)

        path_msg = Path()
        path_msg.header.frame_id = "/map"
        path_msg.header.stamp = rospy.Time.now()

        output_path.reverse()

        for index in range(len(output_path) - 1):
            # point (x, y)
            vector_start = (int(output_path[index][0]) * lm_info.resolution + lm_info.origin.position.x, int(output_path[index][1]) * lm_info.resolution + lm_info.origin.position.y)
            vector_end = (int(output_path[index + 1][0]) * lm_info.resolution + lm_info.origin.position.x, int(output_path[index + 1][1]) * lm_info.resolution + lm_info.origin.position.y)

            ps = PoseStamped()
            ps.header.frame_id = "/map"
            ps.header.stamp = rospy.Time.now()

            ps.pose.position.x = vector_start[0]
            ps.pose.position.y = vector_start[1]

            yaw = atan2(vector_end[1] - vector_start[1], vector_end[0] - vector_start[0])
            quat = quaternion_from_euler(0, 0, yaw)
            ps.pose.orientation.x = quat[0]
            ps.pose.orientation.y = quat[1]
            ps.pose.orientation.z = quat[2]
            ps.pose.orientation.w = quat[3]

            path_msg.poses.append(ps)
            


        rospy.loginfo("path published to topic astar_path")

        self.path_publisher.publish(path_msg)
        return getAstarPathResponse(True)

#defining the heuristics for distance
def heuristic(a, b):
    return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

#fucntion to calculate the path provided the start and end goal of the robot as an input
def astar(array, start, goal):

    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False

if __name__ == "__main__":
    rp = GetMap()
