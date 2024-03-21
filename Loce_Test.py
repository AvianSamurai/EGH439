import argparse
import time
import cv2
import subprocess, sys
import random
import numpy as np
import matplotlib.pyplot as plt
from pibot_client import PiBot
from runner import Runner
import win32com.client as wincom
import os

# [===============================[ SETTINGS ]==================================]
# Connection settings
IP = "172.19.232.146"
USE_LOCALIZER = True;
LOCALIZER_NUM = 2;
print("Connecting to the bot\n")
bot = PiBot(ip=IP, localiser_ip=f'egb439localiser{LOCALIZER_NUM}')
CurrPose = [0, 0, 0]
i = 0
StartTime = time.time()


def LoceSpeed():
    Pose = bot.getLocalizerPose(9)
    
    i = i + 1
    if Pose == CurrPose:
        loceOutputState = 'Dupe'
    elif not Pose == CurrPose:
        loceOutputState = 'New'
        CurrPose = Pose
    else:
        loceOutputState = 'Error'
        Pose = [0, 0, 0]
    CurrTime = time.time() - StartTime
    return(i, CurrTime, loceOutputState, Pose)

while (time.time() - StartTime) < 20:
    print(f"{LoceSpeed()}")