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
USE_TRACKER = False;
LOCALIZER_NUM = 1;

# Run Type
# 0 = Line
# 1 = Figure 8
RUN_TYPE = 0;
FIGURE_8_TIME = 20;

# Robot Properties
WHEEL_RADIUS = 65.7/2 # mm
WIDTH = 150/1000 # m
M_PER_1_PER_S = 5.35/1000;

# Pose Based Settings
K_A = 4.7;
K_B = 2;
K_P = 0.25;
K_TOTAL = 1.5;

# Pure Persuit Settings
USE_PURE_PURSUIT = (RUN_TYPE == 1);
USE_POSE_BASED_FOR_FIRST_WP = True;
K_TURN_GAIN = 5;
PURE_PURSUIT_V = 1;

# Robot Settings
TRIGGER_DIST = 0.1;
MAX_TURN_RATE = 20; #30

# Arena Limit settings
MIN_LIM = 0.15;
MAX_LIM = 1.85;

# Debug Settings
RUN_COUNT = 2500; # 0 for off
SHOW_DEBUG_GRAPH = True;
SHOW_MOTOR_COMMANDS = False;
USE_SPEACH = True;

# Debug variables
past_positions_x = [];
past_positions_y = [];
velocity_graph = [];
turn_rate_graph = [];
left_wheel_speeds = [];
right_wheel_speeds = [];

# Speech Lines
speak = 0;
moveToWaySpeak = ["Moving to first waypoint", 
                  "Heading to first waypoint",
                  "Aligning with first waypoint",
                  "On my way to the first waypoint",
                  "Going to the waypoint"]

startingRunSpeak = ["Starting the timer",
                    "Starting the run",
                    "Ok, Lets go",
                    "Time to do some driving",
                    "The timer has started"]

outOfTimeSpeak = ["Out of time, heading to wall",
                  "Timer has elapsed, time to head to the wall",
                  "Thats all the time we have for today, Parking at the wall",
                  "No time left, wall time",
                  "Heading to the wall as the timer has run dry"]

completeSpeak = ["All done", 
                 "Completed", 
                 "Did i make you proud?", 
                 "That was a good run, right?",
                 "Did you see that?"]

def randLine(lineArray):
    return random.choice(lineArray);

# [===============================[ METHODS ]==================================]

def transformation_matrix_2d(x, y, theta):
    return np.array([[np.cos(theta),   -np.sin(theta), x],
                     [np.sin(theta),    np.cos(theta), y],
                     [0,                0,             1]], 
                    dtype=np.float64);

def clamp(val, minVal, maxVal): return max(min(val, maxVal), minVal);

def drive(speed, turn_rate):

    V_right = ((turn_rate*WIDTH) - (2*speed))/(-2);
    V_left = (2*speed) - V_right;

    right_wheel = V_left/M_PER_1_PER_S;
    left_wheel = V_right/M_PER_1_PER_S;

    if(left_wheel > 100):
        right_wheel = right_wheel/left_wheel * 100;
        left_wheel = 100;
    if(right_wheel > 100):
        left_wheel = left_wheel/right_wheel * 100;
        right_wheel = 100;
    
    if(SHOW_DEBUG_GRAPH):
        left_wheel_speeds.append(left_wheel);
        right_wheel_speeds.append(right_wheel);

    if(SHOW_MOTOR_COMMANDS):
        print(f'v: {round(speed, 3)}, turn_rate: {round(turn_rate, 2)}')
        print(f'\t => {round(right_wheel, 2)} {round(left_wheel,2)}')

    left_wheel = round(left_wheel);
    right_wheel = round(right_wheel) 

    if(USE_LOCALIZER):
        bot.setVelocity(motor_left=min(max(left_wheel, -100), 100), motor_right=min(max(right_wheel, -100), 100), duration=10, acceleration_time=None);
    else:
        visualizer.setVelocity(min(max(left_wheel, -100), 100), min(max(right_wheel, -100), 100), 0.1);
    time.sleep(0.01);

def GetPoints():
    ##get robot pose
    pose = bot.getLocalizerPose(5) if USE_LOCALIZER else visualizer.getPose();
    RobX, RobY, = pose[0], pose[1]

    ##Rearrange the equation in the form ax + by + c = 0
    a = -1
    b = 1
    c = 0.001
    x = np.linspace(0, 2, 1000)
    y = (-a * x - c) / b


    ##plot line
    fig1 = plt.figure()
    plt.axis([0, 2, 0, 2])
    plt.plot(x, y, linestyle = '-')

    ##plot robto
    plt.plot(RobX,RobY,'ro') 
    #plt.show

    ##find closest point
    ClosestX = (b * (b * RobX - a * RobY) - a * c) / (a**2 + b**2)
    ClosestY = (a * (-b * RobX + a * RobY) - b * c) / (a**2 + b**2)
    ClosestPoint = ClosestX, ClosestY

    plt.plot(ClosestX, ClosestY,'go') 
    #plt.show()

    ##find end point
    theta = np.pi/4;
    if ClosestX < 1:
        #EndPoint = 2, ((-a * 2 - c) / b)
        EndPoint = 2, 2
    else:
        #
        EndPoint = 0.01, 0.01
        theta = -3*(np.pi/4)

    plt.plot(EndPoint[0], EndPoint[1],'bo') 
    #plt.show()

    PathX = np.linspace(ClosestX, EndPoint[0], 8)
    PathY = np.linspace(ClosestY, EndPoint[1], 8)
    PathT = np.ones((len(PathY))) * theta;
    PathX = PathX[2:7]
    PathY = PathY[2:7]

    plt.plot(PathX, PathY,'o') 
    plt.arrow(RobX, RobY, 0.1*np.cos(pose[2]*(np.pi/180)), 0.1*np.sin(pose[2]*(np.pi/180)))
    plt.show()

    return (PathX, PathY, PathT, 0)

def GetFig8():
    ##get robot pose
    pose = GetPose();
    RobX, RobY, = pose[0], pose[1]
    Num = 25
    NumSpec = 100

    ##generate path
    t = np.linspace(0, 2*np.pi, Num)
    tLine = np.linspace(0, 2*np.pi, NumSpec)
    a = 0.75 / np.sqrt(2)

    PathX = (a * np.sqrt(2) * np.cos(t)) / (np.sin(t)**2 + 1) + 1
    PathY = (a * np.sqrt(2) * np.cos(t) * np.sin(t)) / (np.sin(t)**2 + 1) + 1

    xLine = (a * np.sqrt(2) * np.cos(tLine)) / (np.sin(tLine)**2 + 1) + 1
    yLine = (a * np.sqrt(2) * np.cos(tLine) * np.sin(tLine)) / (np.sin(tLine)**2 + 1) + 1

    print(PathX)
    print(PathY)

    ##plot path
    fig1 = plt.figure()
    plt.axis([0, 2, 0, 2])
    plt.plot(PathX, PathY, 'bo')
    plt.plot(xLine, yLine, 'k-')
    #plt.show()

    ##plot robto
    plt.plot(RobX,RobY,'ro') 
    plt.arrow(RobX, RobY, 0.1*np.cos(pose[2]*(np.pi/180)), 0.1*np.sin(pose[2]*(np.pi/180)))
    plt.show()

    ##find closest point
    i = 0
    j = 0
    CloPeak = 100
    while i < Num: 
        CloMeas = np.sqrt((RobX - PathX[i])**2 + (RobY - PathY[i])**2)
        if CloPeak > CloMeas:
            j = i
            CloPeak = CloMeas
        i = i + 1
    ClosestPoint = PathX[j], PathY[j]
    Indx = j

    #plt.plot(ClosestPoint[0], ClosestPoint[1],'go') 
    #plt.show()

    PathX[:0] = [ClosestPoint[0]]
    PathY[:0] = [ClosestPoint[1]]

    ##generate poses
    PathT = [];
    for i in range(0, len(PathX)):
        if(i >= len(PathX) - 1):
            PathT.append(np.arctan2(PathY[0] - PathY[i], PathX[0] - PathX[i]))
        else:
            PathT.append(np.arctan2(PathY[i+1] - PathY[i], PathX[i+1] - PathX[i]))
        plt.arrow(PathX[i], PathY[i], 0.1*np.cos(PathT[i]), 0.1*np.sin(PathT[i]))

    plt.axis([0, 2, 0, 2])
    plt.plot(PathX, PathY,'bo') 
    plt.plot(ClosestPoint[0], ClosestPoint[1],'go') 
    plt.plot(RobX, RobY,'ro') 
    plt.show()

    return (PathX, PathY, PathT, Indx);

last_pose = [];
last_time = 0;
start_time = 0;

def RecordDebugData(pose, steering_angle):
    global last_time, start_time, last_pose;

    if(SHOW_DEBUG_GRAPH):
        past_positions_x.append(pose[0]);
        past_positions_y.append(pose[1]);
        turn_rate_graph.append(steering_angle);

        if(last_pose == []): last_pose = GetPose();

        pose = GetPose();
        pose_dist = np.sqrt((pose[0] - last_pose[0])**2 + (pose[1] - last_pose[1])**2);
        last_pose = pose;
        delta_t = time.time() - last_time;
        last_time = time.time();
        velocity_graph.append(pose_dist / delta_t);

def StartRun():
    global last_time, start_time, last_pose;

    print("Run Starting");

    if(USE_SPEACH):
        speak.Speak(randLine(startingRunSpeak));

    last_time = time.time();
    start_time = time.time();
    last_pose = GetPose();

def GetPose():
    return bot.getLocalizerPose(9) if USE_LOCALIZER else visualizer.getPose();

def DriveToWall(velocity):
    pose = GetPose();
    while(pose[0] < MAX_LIM and pose[1] < MAX_LIM and pose[0] > MIN_LIM and pose[1] > MIN_LIM):
        pose = GetPose();
        drive(velocity, 0);
        RecordDebugData(pose, 0);

def PoseBased(pose, theta, wp):
    # using control scheme proposed in slide 47 of Lecture 3, calculate a, b, p
    a = np.arctan2(wp[1],wp[0]);
    b = pose[2] - theta
    p = np.sqrt(wp[0]**2 + wp[1]**2);

    # calculate velocity, direction (L), and heading angle
    v = clamp(p*K_P, 0.3, 0.5);
    w = K_A*a + K_B*b;

    # Convert the heading angle to strearing rate
    steering_angle = clamp(np.arctan((w)/v), -np.pi, np.pi) * K_TOTAL;

    return (v, steering_angle)

def PurePersuit(wp):
    # Convert the heading angle to strearing rate
    steering_angle = np.arctan2(wp[1],wp[0]) * K_TURN_GAIN;

    return (PURE_PURSUIT_V, steering_angle)

def StepTowardsPosition(x, y, t, usePurePursuit):
    # Get the pose of the robot and convert to radians
    rawPose = GetPose();
    pose = [rawPose[0], rawPose[1], rawPose[2] * (np.pi/180)];

    # Calculate transform of world with respect to the robot
    T_RW = np.linalg.inv(transformation_matrix_2d(pose[0], pose[1], pose[2]));

    # Create a vector representing the waypoint and transform it into robots coordinate
    # frame to get an offset from the robot
    wp = np.transpose(T_RW @ np.array([[x], [y], [1]] ,dtype=np.float64))[0];

    v, steering_angle = PurePersuit(wp) if usePurePursuit else PoseBased(pose, t, wp);

    # Instruct the car to drive at the speed and turnrate
    drive(v, steering_angle);

    # Debug stuff
    RecordDebugData(pose, steering_angle);

    # Return the waypoint position relative to the robot
    return wp;

# [===============================[ MAIN ]==================================]

if __name__ == "__main__":
    speak = 0;
    if USE_SPEACH:
        speak = wincom.Dispatch("SAPI.SpVoice")

    # Forge an unbreakable connection
    if(USE_LOCALIZER):
        if USE_TRACKER: subprocess.Popen(f"cmd /c python LiveTrack2.py --localizer {LOCALIZER_NUM} --IP {IP}", cwd=os.getcwd(), shell=True);
        print("Connecting to the bot\n")
        bot = PiBot(ip=IP, localiser_ip=f'egb439localiser{LOCALIZER_NUM}')
    else:
        visualizer = Runner();
        visualizer.randomizeStartingPose();
        print("Connecting to the bot without localizer\n")

    # Make sure the connection exists and that its not lieing to me
    if(USE_LOCALIZER):
        print("Running tests")
        print(f'\tVoltage: {bot.getVoltage():.2f}V')
        print(f'\tCurrent: {bot.getCurrent():.2f}A')

    # Some ned shit
    points = GetPoints() if RUN_TYPE == 0 else GetFig8();

    # debug stuff
    visited_x = [];
    visited_y = [];

    # Setup some time sensitive run variables
    if RUN_TYPE == 0: StartRun();

    # Do the go
    index = points[3]; # Waypoint Index
    itterations = -1; # Itteration count for limiting run time
    isFirstWp = True;
    print(f'First Waypoint: x:{points[0][0]}, y:{points[1][0]}')
    if USE_SPEACH:
        speak.Speak(randLine(moveToWaySpeak));
    while(index < len(points[0]) and itterations < RUN_COUNT):
        use_ps = USE_PURE_PURSUIT
        if(USE_POSE_BASED_FOR_FIRST_WP and isFirstWp):
            use_ps = False;
        
        way_pos = StepTowardsPosition(points[0][index], points[1][index], points[2][index], use_ps);

        # Calculate distance to waypoint so see if waypoint is triggered
        dist = np.sqrt((way_pos[0])**2 + (way_pos[1])**2)
        if(dist < TRIGGER_DIST): # If true waypoint has triggered
            index = index + 1;
            if(index < len(points[0])):
                if(isFirstWp and RUN_TYPE == 1):
                    # Setup some time sensitive run variables
                    StartRun();
                isFirstWp = False;
                print(f'TRIGGERED: Next Waypoint: x:{points[0][index]}, y:{points[1][index]}')
                print(f'\t Dist was {round(dist,2)} for wp_x: {round(way_pos[0], 2)} wp_y: {round(way_pos[1], 2)}')

        if(RUN_TYPE == 1):
            if(index >= len(points[0]) - 1):
                index = 0;
            if(not isFirstWp and time.time() - start_time > FIGURE_8_TIME):
                print("Time limit reached, Driving to wall")
                if(USE_SPEACH):
                    speak.Speak(randLine(outOfTimeSpeak));
                break;


        # If theres a run count limitation, keep track of itterations
        if(RUN_COUNT > 0):
            itterations = itterations + 1;
    
    if(itterations >= RUN_COUNT):
        print("Itteration limit reached, ending early")

    print("Driving to wall");
    DriveToWall(0.5);
    print("Done");

    if(USE_SPEACH):
        speak.Speak(randLine(completeSpeak));
    
    # Stop the wheels from moving afterwards
    if(USE_LOCALIZER):
        bot.setVelocity(motor_left=0, motor_right=0, duration=None, acceleration_time=None)

    # Graph the final run
    if(SHOW_DEBUG_GRAPH):
        fig0, ax0 = plt.subplots(2,2);
        # Map Plot
        ax0[0,0].axis([0, 2, 0, 2]);
        ax0[0,0].set_title(f'Hit Points: {index}');
        ax0[0,0].plot(past_positions_x, past_positions_y);
        ax0[0,0].plot(past_positions_x[len(past_positions_x) - 1], past_positions_y[len(past_positions_y) - 1], "x")
        ax0[0,0].scatter(points[0], points[1], c = ("b" if index < len(points[0]) else "g"));
        if(index > 0 and index < len(points[0])):
            ax0[0,0].scatter(points[0][0:index], points[1][0:index], c="g");
        
        for i in range(len(points[0])):
            ax0[0,0].add_patch(plt.Circle((points[0][i], points[1][i]), TRIGGER_DIST, color='b', linestyle='--', fill=False));
        ax0[0,0].set_aspect('equal', 'box');
    
        # Turnrate Plot
        ax0[1,0].plot(turn_rate_graph);
        ax0[1,0].set_title("Turn rate over time")

        # velocity plot
        ax0[0,1].plot(velocity_graph);
        avg_velocity = np.mean(velocity_graph);
        ax0[0,1].set_title(f"Velocity over time, Mean: {round(avg_velocity,3)}");

        ax0[1,1].plot(left_wheel_speeds);
        ax0[1,1].plot(right_wheel_speeds);
        ax0[1,1].legend(["Left Wheel", "Right Wheel"]);
        ax0[1,1].set_title("Wheel Speeds");

        plt.show();