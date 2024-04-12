#Setup
import argparse
import time
import cv2
from pibot_client import PiBot
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description='PiBot client')
parser.add_argument('--ip', type=str, default='localhost', help='IP address of PiBot')
args = parser.parse_args()
bot = PiBot(ip=args.ip)

RobX, RobY, RobTheta = bot.getLocalizerPose()

#Rearrange the equation in the form ax + by + c = 0
a = 1
b = 1
c = 1
x = np.linspace(0, 2000, 1000)
y = (-a * x - c) / b


#plot line
plt.plot(x, y, linestyle = '-')
plt.show()
plt.axis([0, 2000, 0, 2000])


