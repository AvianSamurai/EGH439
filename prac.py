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

# Robot Properties
WHEEL_RADIUS = 65.7/2 # mm
WIDTH = 150/1000 # m

# Robot Settings
K_A = 4.7;
K_B = -2;
K_P = 0.25;
M_PER_1_PER_S = 5.35/1000;
offset = 0.1;

# Robot Settings
TRIGGER_DIST = 0.1;
TRIGGER_THETA = 5 * (np.pi/180)
ROBOT_V = 50;
INTERNAL_TURN_GAIN = 2;
POSE_TURN_GAIN = 1;
TURN_GAIN = 15;
MAX_TURN_RATE = 30;

# Arena Limit settings
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

    if(SHOW_MOTOR_COMMANDS):
        print(f'v: {round(speed, 3)}, turn_rate: {round(turn_rate, 2)}')
        print(f'\t => {round(right_wheel, 2)} {round(left_wheel,2)}')

    if(USE_LOCALIZER):
        bot.setVelocity(motor_left=min(max(left_wheel, -100), 100), motor_right=min(max(right_wheel, -100), 100), duration=None, acceleration_time=None);
    else:
        visualizer.setVelocity(min(max(left_wheel, -100), 100), min(max(right_wheel, -100), 100), 0.1);

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

    return (PathX, PathY, PathT)

def GoToPosition(x, y, t):
    # Get the pose of the robot and convert to radians
    pose = bot.getLocalizerPose(9) if USE_LOCALIZER else visualizer.getPose();
    pose[2] = pose[2] * (np.pi/180);

    # Calculate transform of world with respect to the robot
    T_RW = np.linalg.inv(transformation_matrix_2d(pose[0], pose[1], pose[2]));

    # Create a vector representing the waypoint and transform it into robots coordinate
    # frame to get an offset from the robot
    wp = np.transpose(T_RW @ np.array([[x], [y], [1]] ,dtype=np.float64))[0];

    # using control scheme proposed in slide 47 of Lecture 3, calculate a, b, p
    a = np.arctan2(wp[1],wp[0]);
    b = pose[2] - t
    p = np.sqrt(wp[0]**2 + wp[1]**2);

    # calculate velocity, direction (L), and heading angle
    v = clamp(p*K_P, 0.25, 0.5);
    dir = 1 #if wp[0] >= 0 else -1;
    w = K_A*a + K_B*b;

    # Convert the heading angle to strearing rate
    steering_angle = clamp(np.arctan((w*dir)/v), -np.pi/2, np.pi/2);

    # Instruct the car to drive at the speed and turnrate
    drive(v*dir, steering_angle);

    # Debug stuff
    if(SHOW_FINAL_GRAPH):
        past_positions_x.append(pose[0]);
        past_positions_y.append(pose[1]);
    if(SHOW_TURN_GRAPH):
        turn_rate_graph.append(steering_angle);

    # Return the waypoint position relative to the robot
    return wp;

def GoToPositionOld(x, y, t):
    # Get the post of the robot, if the localizer isn't running, that just make
    # some shit up so i can test the fucker
    pose = bot.getLocalizerPose(9) if USE_LOCALIZER else visualizer.getPose();
    bot_t = pose[2] * (np.pi/180);

    # Work out coordinates of waypoint in reference to local coordinate frame
    T_WR = transformation_matrix_2d(pose[0], pose[1], bot_t);
    waypoint_pos = np.linalg.inv(T_WR) @ np.transpose(np.array([x, y, 1], dtype=np.float64));
    waypoint_pos = np.transpose(waypoint_pos)

    a = np.arctan2(waypoint_pos[1], waypoint_pos[0]);
    b = -bot_t - t;

    # Work out angle from front of robot to waypoint, wrap it to range [-90, 90]
    w = a * INTERNAL_TURN_GAIN + b * POSE_TURN_GAIN;
    #v = np.sqrt(waypoint_pos[0]**2 + waypoint_pos[1]**2);
    direction = 1 if waypoint_pos[0] >= 0 else -1;
    turn_rate = np.arctan(w*direction);
    turn_rate = clamp(TURN_GAIN*turn_rate, -MAX_TURN_RATE, MAX_TURN_RATE)
    #turn_rate = (np.arctan2(waypoint_pos[1], waypoint_pos[0]) / np.pi) * TURN_GAIN;
    if(SHOW_TURN_GRAPH):
        turn_rate_graph.append(turn_rate);

    # Work out how fast the wheels should be turning
    left_wheel = round(direction*(-(turn_rate) + ROBOT_V));
    right_wheel = round(direction*(turn_rate + ROBOT_V));
    if(SHOW_MOTOR_COMMANDS):
        print(f'Pose x:{round(pose[0], 2)} y:{round(pose[1], 2)} t:{round(bot_t, 2)}, WP:x:{round(waypoint_pos[0], 2)} y:{round(waypoint_pos[1], 2)} t:{round(waypoint_pos[2], 2)}')
        print(f'({turn_rate}) => {right_wheel} {left_wheel}')

    # Make bot go broooom brooom
    if(USE_LOCALIZER):
        bot.setVelocity(motor_left=min(max(left_wheel, -100), 100), motor_right=min(max(right_wheel, -100), 100), duration=None, acceleration_time=None);
    else:
        visualizer.setVelocity(min(max(left_wheel, -100), 100), min(max(right_wheel, -100), 100), 0.2);

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
        way_pos = GoToPosition(points[0][index], points[1][index], points[2][index]);
        dist = np.sqrt((way_pos[0])**2 + (way_pos[1])**2)
        if(dist < TRIGGER_DIST):# and abs(np.arctan2(way_pos[1],way_pos[0])) < TRIGGER_THETA):
            index = index + 1;
            if(index < len(points[0])):
                print(f'TRIGGERED: Next Waypoint: x:{points[0][index]}, y:{points[1][index]}')
                print(f'\t Dist was {round(dist,2)} for wp_x: {round(way_pos[0], 2)} wp_y: {round(way_pos[1], 2)}')
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
        ax0 = plt.gca();
        plt.axis([0, 2, 0, 2]);
        plt.title(f'Hit Points: {index}')
        plt.plot(past_positions_x, past_positions_y);
        plt.plot(past_positions_x[len(past_positions_x) - 1], past_positions_y[len(past_positions_y) - 1], "x")
        plt.scatter(points[0], points[1], c = ("b" if index < len(points[0]) else "g"));
        if(index > 0 and index < len(points[0])):
            plt.scatter(points[0][0:index], points[1][0:index], c="g");
        
        for i in range(len(points[0])):
            ax0.add_patch(plt.Circle((points[0][i], points[1][i]), TRIGGER_DIST, color='b', linestyle='--', fill=False));
        ax0.set_aspect('equal', 'box');
        plt.show();
    
    if(SHOW_TURN_GRAPH):
        fig1 = plt.figure();
        plt.plot(turn_rate_graph);
        plt.title("Turn rate over time")
        plt.show();