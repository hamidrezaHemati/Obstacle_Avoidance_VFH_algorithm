#!/usr/bin/python3

from cmath import inf
from dis import dis
from math import degrees, dist
from re import A, M
from tkinter.tix import Tree
import rospy
import tf

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


from matplotlib import pyplot as plt
import numpy as np
from matplotlib import colors
from matplotlib.ticker import PercentFormatter

from math import atan2,sqrt,pow
import pandas as pd


class Robot():
    def __init__(self):
        rospy.init_node("robot", anonymous=False)
        rospy.on_shutdown(self.on_shutdown)

        self.cmd_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom = rospy.Subscriber("/odom", Odometry, callback=self.update_pose, queue_size=10)


        ## target position
        self.target = Point()
        self.target.x = -4
        self.target.y = 0

        ## robot position
        self.x = -3
        self.y = 0
        self.yaw = 0

        ## VFH algorithms parameters
        self.a = 0.875
        self.b = 0.25
        self.d_max = 3.5 #meters
        self.alpha = 5
        self.k = int(360/self.alpha) #number of sectors
        self.l = 2 #satisfactory smothing 
        self.threshold = 3

        self.dt = 0.1
        self.rate = rospy.Rate(1/self.dt)


    def quaternion_to_euler(self, msg):
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        return yaw
    

    def update_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_euler(msg)


    def vision(self):
        laser_data = rospy.wait_for_message("/scan", LaserScan)
        laser_data = laser_data.ranges[0:359]
        return laser_data
        
        
    ## m = (ci,j)^2 * (a - b*di,j)
    def magnitude_calculator(self, c):
        m = [None]*len(c)
        for degree, dist in enumerate(c):
            value = 0
            if dist != inf:
                value = self.a - (self.b * dist)
            m[degree] = value
        return m


    ## polar obstacle density --- h_k
    ## h_k = sigma(m) 
    ## k is number of sectors and is equal to degrees/number of degrees in each sector --> 360/5 = 72
    def pod(self, m):
        h = [0] * self.k
        i = 0
        for degree, value in enumerate(m):
            if int(degree/self.alpha) != i:
                i += 1
            h[i] += value
        return h

    
    ## smothed_h
    def smothed_pod(self, h):
        smothed_h = [None]*self.k
        for i in range(self.k):
            if i == 70:
                smothed_h[i] = ( h[i-self.l] + 2*h[i-self.l+1] + 2*h[i] + 2*h[i + self.l - 1] + h[0] ) / (2*self.l + 1)
            elif i == 71:
                smothed_h[i] = ( h[i-self.l] + 2*h[i-self.l+1] + 2*h[i] + 2*h[0] + h[1] ) / (2*self.l + 1)
            else:
                smothed_h[i] = ( h[i-self.l] + 2*h[i-self.l+1] + 2*h[i] + 2*h[i + self.l - 1] + h[i + self.l] ) / (2*self.l + 1)
        return smothed_h


    def bar_plot(self, data):
        data = np.array(data)
        index2 = np.array([i for i in range(72)])

        df = pd.DataFrame(list(zip(data, index2)), columns=['data', 'index'])

        mask1 = data < self.threshold
        mask2 = data >= self.threshold

        plt.bar(index2[mask1], data[mask1], color = 'green')
        plt.bar(index2[mask2], data[mask2], color = 'red')
        plt.show()


    def bar_plot_circular(self, data):
        data = np.array(data)
        index = np.array([i for i in range(72)])

        df = pd.DataFrame(
                {
                    'index': index,
                    'data': data
                })

        plt.figure(figsize=(20,10))
        # plot polar axis
        ax = plt.subplot(111, polar=True)
        # remove grid
        plt.axis('off')
        upperLimit = max(data)
        lowerLimit = 0
        _max = df['data'].max()
        slope = (_max - lowerLimit) / _max
        heights = slope * df.data + lowerLimit
        # Compute the width of each bar. In total we have 2*Pi = 360°
        width = 2*np.pi / len(df.index)
        # Compute the angle each bar is centered on:
        indexes = list(range(1, len(df.index)+1))
        angles = [element * width for element in indexes]

        bars = ax.bar(
            x=angles,
            height=heights,
            width=width,
            bottom=lowerLimit,
            linewidth=2,
            edgecolor="white")
        # Add labels
        for bar, angle, height, label in zip(bars,angles, heights, df["index"]):

            # Labels are rotated. Rotation must be specified in degrees :(
            rotation = np.rad2deg(angle)

            # Flip some labels upside down
            alignment = ""
            if angle >= np.pi/2 and angle < 3*np.pi/2:
                alignment = "right"
                rotation = rotation + 180
            else:
                alignment = "left"

            # Finally add the labels
            ax.text(
                x=angle,
                y=lowerLimit + bar.get_height(),
                s=label,
                ha=alignment,
                va='center',
                rotation=rotation,
                rotation_mode="anchor")
        plt.show()


    def target_angular_difference(self):
        inc_x = self.target.x - self.x
        inc_y = self.target.y - self.y
        theta = atan2(inc_y, inc_x)
        return int(degrees(theta))


    ## this method changes the origin of the trigonometric-circle to be fit with lidar sensor trigonometric_circle
    ## in trigonometric_circle 0 is right but in lidar 0th sensor is front of the robot
    def shift_origin_of_trigonometric_circle(self, degree):
        shifted_degree = int(degree)
        if 90 <= int(degree) <= 180:
            shifted_degree = int(degree) - 90
        else:
            shifted_degree = int(degree) + 270
        return shifted_degree

    
    ## this method converts the sensor heading to trigonometric circle theta
    def get_trigonometric_circle_theta(self, sector_num):
        heading = sector_num * 5 + 2
        theta = 0
        if 0 <= heading <= 90:
            theta = heading + 90
        elif 91 <= heading <= 360:
            theta = heading - 270
        return theta


    def heading_to_target(self, angular_deffrences_with_target):
        shifted_theta = self.shift_origin_of_trigonometric_circle(angular_deffrences_with_target)
        shifted_yaw = self.shift_origin_of_trigonometric_circle(degrees(self.yaw))
        _heading = shifted_theta - shifted_yaw
        if shifted_yaw > shifted_theta:
            _heading += 360
        print("target heading: ", _heading)
        return _heading

    
    ## returns sector number of target
    def get_target_sector(self, target_heading):
        return int(target_heading/self.alpha)

    
    ## this method finds valleys according to VFH algorithm
    ## returns a list of vallyes with their sector numbers
    def valley_finder(self, smothed_h):
        rows, cols = (72, 2)
        arr = [[0 for i in range(cols)] for j in range(rows)]
        for i,v in enumerate(smothed_h):
            arr[i][0] = i
            arr[i][1] = v

        valleys = []
        flag = False
        counter = 0
        while True:
            new_valley = []
            if arr[counter][1] < self.threshold:
                new_valley.append(arr[counter])
                for j in range(counter + 1, len(arr)):
                    if arr[j][1] < self.threshold:
                        new_valley.append(arr[j])
                    else:
                        flag = True
                        break
                    flag = True
            if flag is True:
                counter = new_valley[-1][0] + 1
                valleys.append(new_valley)
                flag = False
            else:
                counter += 1
            if counter == len(arr):
                break

        valleys_sector_numbers = []
        for valley in valleys:
            new = []
            for v in valley:
                new.append(v[0])
            valleys_sector_numbers.append(new)

        return valleys_sector_numbers


    def circular_dist(self,lenght_of_list, idx_1,idx_2):
        i = (idx_1 - idx_2) % lenght_of_list
        j = (idx_2 - idx_1) % lenght_of_list
        return min(i, j)


    ## this method returns the desiered theta, theta is a sector number
    ## theta is the angle that the robot should take to get to the target
    ## theta is in a valley or is the average of the bst valley
    def find_theta(self,valleys, target_sector):
        is_in_valley = False
        for valley in valleys:
            if target_sector in valley:
                is_in_valley = True
                return target_sector   ## target is in a valley and it is reachable

        if not is_in_valley:
            corners = []
            for valley in valleys:
                corner = [valley[0], valley[-1]]
                corners.append(corner)

            distance_from_corners = []
            for corner in corners:
                new = [self.circular_dist(72, target_sector, corner[0]), self.circular_dist(72, target_sector, corner[1])]
                distance_from_corners.append(new)

            minimum_distances_from_corners = []
            for d in distance_from_corners:
                minimum_distances_from_corners.append([distance_from_corners.index(d), min(d)])

            dummy = min([x[1] for x in minimum_distances_from_corners])
            chosen_valley = 0
            for d in minimum_distances_from_corners:
                if d[1] == dummy:
                    chosen_valley = d[0]
                    break
            kn = valleys[chosen_valley][0]
            kf = valleys[chosen_valley][-1]
            return int(kn+kf)/2


    def euclidean_distance(self, point_1, point_2):
        x_1, y_1 = point_1
        x_2, y_2 = point_2
        dx = (x_2 - x_1)
        dy = (y_2 - y_1)
        return sqrt(pow(dx, 2) + pow(dy, 2))


    def move_to_target(self,theta):
        speed = Twist()

        distance_error = self.euclidean_distance((self.x, self.y), (self.target.x, self.target.y))

        # inc_x = self.goal.x - self.x
        # inc_y = self.goal.y - self.y
        # angle_to_goal = atan2(inc_y, inc_x)

        if distance_error > 0.08:
            if abs(theta - self.yaw) > 0.15:
                print("rotating")
                speed.linear.x = 0
                speed.angular.z = -0.25
            else:
                print("moving to point")
                speed.linear.x = 0.2
                speed.angular.z = 0
        else:
            speed.linear.x = 0
            speed.angular.z = 0

        return speed


    def run(self):
        rospy.sleep(2)
        twist = Twist()

        while not rospy.is_shutdown():

            ## calculating polar obstacle density (c,m,h,h')
            c = self.vision()
            m = self.magnitude_calculator(c)
            h = self.pod(m)
            smothed_h = self.smothed_pod(h)
            # self.bar_plot(smothed_h)
            # self.bar_plot_circular(smothed_h)


            ## finding robot heading(in lidar sensor order) towards the target
            angular_deffrences_with_target = self.target_angular_difference()
            target_heading = self.heading_to_target(angular_deffrences_with_target)
            target_sector = self.get_target_sector(target_heading)


            ## finding theta using heading and smothed_h
            valleys = self.valley_finder(smothed_h)
            print(valleys)
            theta_sector = self.find_theta(valleys, target_sector)
            print("heading sector ", theta_sector)
            theta = self.get_trigonometric_circle_theta(theta_sector)
            print("heading theta ", theta)


            ## moving the robot
            twist = self.move_to_target(theta)
            self.cmd_publisher.publish(twist)

            self.rate.sleep()
            
   
    def on_shutdown(self):
        rospy.sleep(1)
        

if __name__ == "__main__":
    robot = Robot()
    robot.run()