import argparse
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pibot_client import PiBot
from runner import Runner

# [===============================[ SETTINGS ]==================================]
# Connection settings
IP = "172.19.232.146"
USE_LOCALIZER = False;
LOCALIZER_NUM = 2;

# Encoder Settings
LEFT_ENC = 200;
RIGHT_ENC = 200;

# Robot Settings
TRIGGER_DIST = 0.15;
ROBOT_V = 50;
TURN_GAIN = 30;
POSE_TURN_GAIN = 30;

# Limit settings
MIN_LIM = 0.15;
MAX_LIM = 1.85;

# Debug Settings
RUN_COUNT = 2500; # 0 for off
SHOW_FINAL_GRAPH = True;
SHOW_TURN_GRAPH = True;
SHOW_MOTOR_COMMANDS = False;

# Debug variables
past_positions_x = [];
past_positions_y = [];
turn_rate_graph = [];

# [===============================[ METHODS ]==================================]

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
    if ClosestX < 1:
        #EndPoint = 2, ((-a * 2 - c) / b)
        EndPoint = 2, 2
    else:
        #
        EndPoint = 0.01, 0.01

    plt.plot(EndPoint[0], EndPoint[1],'bo') 
    #plt.show()

    PathX = np.linspace(ClosestX, EndPoint[0], 8)
    PathY = np.linspace(ClosestY, EndPoint[1], 8)
    PathX = PathX[2:7]
    PathY = PathY[2:7]

    plt.plot(PathX, PathY,'o') 
    plt.show()

    return (PathX, PathY)

def GoToPosition(x, y, t):
    # Get the post of the robot, if the localizer isn't running, that just make
    # some shit up so i can test the fucker
    pose = bot.getLocalizerPose(9) if USE_LOCALIZER else visualizer.getPose();
    bot_t = pose[2] * (np.pi/180);

    # Work out coordinates of waypoint in reference to local coordinate frame
    T_WR = np.array([[np.cos(bot_t),    -np.sin(bot_t), pose[0]],
                    [np.sin(bot_t),    np.cos(bot_t), pose[1]],
                    [0,                    0,                  1   ]], 
                dtype=np.float64)
    waypoint_pos = np.linalg.inv(T_WR) @ np.array([x, y, 1], dtype=np.float64);

    a = np.arctan2(waypoint_pos[1], waypoint_pos[0]) - waypoint_pos[2];
    b = -waypoint_pos[2] - a;

    # Work out angle from front of robot to waypoint, wrap it to range [-90, 90]
    turn_rate = (a / np.pi) * TURN_GAIN + (b / np.pi) * POSE_TURN_GAIN;
    #if(waypoint_pos[0] < 0):
    #    turn_rate = -turn_rate;
    turn_rate = (np.arctan2(waypoint_pos[1], waypoint_pos[0]) / np.pi) * TURN_GAIN;
    if(SHOW_TURN_GRAPH):
        turn_rate_graph.append(turn_rate);

    # Work out how fast the wheels should be turning
    left_wheel = round(-(turn_rate) + ROBOT_V);
    right_wheel = round(turn_rate + ROBOT_V);
    if(SHOW_MOTOR_COMMANDS):
        print(f'Pose x:{round(pose[0], 2)} y:{round(pose[1], 2)} t:{round(bot_t, 2)}, WP:x:{round(waypoint_pos[0], 2)} y:{round(waypoint_pos[1], 2)} t:{round(waypoint_pos[2], 2)}')
        print(f'({turn_rate}) => {right_wheel} {left_wheel}')

    # Make bot go broooom brooom
    if(USE_LOCALIZER):
        bot.setVelocity(motor_left=min(max(left_wheel, 0), 100), motor_right=min(max(right_wheel, 0), 100), duration=None, acceleration_time=None);
    else:
        visualizer.setVelocity(min(max(left_wheel, 0), 100), min(max(right_wheel, 0), 100), 0.2);

    # Debug stuff
    if(SHOW_FINAL_GRAPH):
        past_positions_x.append(pose[0]);
        past_positions_y.append(pose[1]);

    #time.sleep(0.5); # Bed time now (perhaps tune sleep length to localizer time or just do a better job)

    return waypoint_pos;

# [===============================[ MAIN ]==================================]

if __name__ == "__main__":
    # Forge an unbreakable connection
    if(USE_LOCALIZER):
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
    points = GetPoints();

    # debug stuff
    visited_x = [];
    visited_y = [];

    # Do the go
    index = 0;
    itterations = -1;
    print(f'First Waypoint: x:{points[0][0]}, y:{points[1][0]}')
    while(index < len(points[0]) and itterations < RUN_COUNT):
        way_pos = GoToPosition(points[0][index], points[1][index], np.arctan2(points[1][index], points[0][index]));
        dist = np.sqrt((way_pos[0])**2 + (way_pos[1])**2)
        if(dist < TRIGGER_DIST):
            index = index + 1;
            if(index < len(points)):
                print(f'TRIGGERED: Next Waypoint: x:{points[0][index]}, y:{points[1][index]}')
        if(RUN_COUNT > 0):
            itterations = itterations + 1;
    
    pose = bot.getLocalizerPose(9) if USE_LOCALIZER else visualizer.getPose();
    while(pose[0] > MIN_LIM and pose[1] > MIN_LIM and pose[0] < MAX_LIM and pose[1] < MAX_LIM):
        pose = bot.getLocalizerPose(9) if USE_LOCALIZER else visualizer.getPose();
        if(SHOW_FINAL_GRAPH):
            past_positions_x.append(pose[0]);
            past_positions_y.append(pose[1]);
        if USE_LOCALIZER:
            bot.setVelocity(motor_left=100, motor_right=100, duration=None, acceleration_time=None)
        else:
            visualizer.setVelocity(100, 100, 0.4);
    
    # Stop the wheels from moving afterwards
    if(USE_LOCALIZER):
        bot.setVelocity(motor_left=0, motor_right=0, duration=None, acceleration_time=None)

    # Graph the final run
    if(SHOW_FINAL_GRAPH):
        fig0 = plt.figure();
        plt.axis([0, 2, 0, 2]);
        plt.plot(past_positions_x, past_positions_y);
        plt.plot(past_positions_x[len(past_positions_x) - 1], past_positions_y[len(past_positions_y) - 1], "x")
        plt.scatter(points[0], points[1]);
        plt.show();
    
    if(SHOW_TURN_GRAPH):
        fig1 = plt.figure();
        plt.plot(turn_rate_graph);
        plt.show();