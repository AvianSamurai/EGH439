import argparse
import time
import cv2
import random
import numpy as np
import matplotlib.pyplot as plt
from pibot_client import PiBot

VELOCITY_TUNER = 5000;
WHEEL_RADIUS = 65.7/2/1000 # m
WIDTH = 150/1000 # m
M_PER_1_PER_S = 5.35/1000; # m/s @ v = 1;

class Runner(object):
    @classmethod
    def __init__(self):
        print("Runner prepared")
        self.currentPoints_x = [];
        self.currentPoints_y = [];
        self.pose = [0, 0, 0];

    @classmethod
    def randomizeStartingPose(self):
        self.pose = [0, 0, 0];
        self.pose[0] = random.uniform(0.2, 1.8);
        self.pose[1] = random.uniform(0.2, 1.8);
        self.pose[2] = random.uniform(-np.pi, np.pi);

    @classmethod
    def getPose(self):
        return [self.pose[0], self.pose[1], ((((self.pose[2] * 180) / np.pi) + 180) % 360) - 180];
        

    @classmethod
    def setVelocity(self, motorLeft, motorRight, time):
        t_dot = -((motorLeft - motorRight)*M_PER_1_PER_S) / WIDTH;
        v = (1/2)*(motorLeft + motorRight)*M_PER_1_PER_S
        delta_pose = np.array([v*np.cos(self.pose[2]),
                               v*np.sin(self.pose[2]),
                               t_dot], 
                               dtype=np.float64);
        self.pose += delta_pose * time;
        self.currentPoints_x.append(self.pose[0]);
        self.currentPoints_y.append(self.pose[1]);

    @classmethod
    def showFinal(self, points):
        fig0 = plt.figure();
        plt.axis([0, 2, 0, 2]);
        plt.plot(self.currentPoints_x, self.currentPoints_y, "-b");
        plt.scatter(points[0], points[1]);
        plt.show();