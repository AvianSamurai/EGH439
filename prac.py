import time
import numpy as np
import matplotlib.pyplot as plt
from pibrait_client import PiBrait

# [===============================[ SETTINGS ]==================================]
# Connection settings
IP = "192.168.50.5"

# Robot Properties
WHEEL_RADIUS = 65.7/2/1000 # mm
WIDTH = 150/1000 # m
ENCODER_RANGE = 377;
ENCODER_TICK_DIST = (WHEEL_RADIUS * 2 * np.pi) / 377
TRANSMISSION = np.array([ENCODER_TICK_DIST, ENCODER_TICK_DIST])
M_PER_1_PER_S = 5.35/1000;

# Berger Settings
K_LEFT = 1;
K_RIGHT = 1;
STOP_VALUE = 0.85;

# Debug Settings
SHOW_DEBUG_GRAPH = True;
SHOW_MOTOR_COMMANDS = True;

# Debug variables
velocity_graph = [];
turn_rate_graph = [];
left_wheel_speeds = [];
right_wheel_speeds = [];

# [===============================[ METHODS ]==================================]

def clamp(val, minVal, maxVal): return max(min(val, maxVal), minVal);

def drive(v_left, v_right):

    right_wheel = v_left;
    left_wheel = v_right;

    if(left_wheel > 100):
        right_wheel = right_wheel/left_wheel * 100;
        left_wheel = 100;
    if(right_wheel > 100):
        left_wheel = left_wheel/right_wheel * 100;
        right_wheel = 100;
    
    left_wheel = round(left_wheel);
    right_wheel = round(right_wheel);

    bot.setVelocity(left_wheel, right_wheel)
    
    if(SHOW_DEBUG_GRAPH):
        left_wheel_speeds.append(left_wheel);
        right_wheel_speeds.append(right_wheel);

    if(SHOW_MOTOR_COMMANDS):
        print("===[ MOTOR COMMAND ]=========================")
        print(f"\tMotor Speeds: [{v_left}, {v_right}]")
        if(right_wheel != v_right or left_wheel != v_left):
            print(f"\tAddjusted to: [{left_wheel}, {right_wheel}]")

last_time = 0;
start_time = 0;

def RecordDebugData(pose, steering_angle):
    global last_time, start_time;

    if(SHOW_DEBUG_GRAPH):
        turn_rate_graph.append(steering_angle);

        pose_dist = np.sqrt((pose[0] - last_pose[0])**2 + (pose[1] - last_pose[1])**2);
        last_pose = pose;
        delta_t = time.time() - last_time;
        last_time = time.time();
        velocity_graph.append(pose_dist / delta_t);

def StartRun():
    global last_time, start_time;

    print("Run Starting");

    last_time = time.time();
    start_time = time.time();

# [===============================[ MAIN ]==================================]

def ShowStats():
    # Graph the final run
    if(SHOW_DEBUG_GRAPH):
        fig0, ax0 = plt.subplots(2,2);
        # Map Plot
        ax0[0,0].set_title(f'Unused Space');
        ax0[0,0].text(1, 1, f"This space is intentionally left blank");
    
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

def BurgCode(): # Dies on true return
    pollen_sensors = bot.sense();

    left_speed = pollen_sensors[1] * K_LEFT;
    right_speed = pollen_sensors[0] * K_RIGHT;
    
    drive(left_speed, right_speed);
    return False;

if __name__ == "__main__":

    # Forge an unbreakable connection
    print(" > Connecting to robot ", end="")
    bot = PiBrait(ip=IP)
    print("[CONNECTED]", end="\n\n")

    # Make sure the connection exists and that its not lieing to me
    print("===[ TESTS ]=================================")
    print(f'\tVoltage: {bot.getVoltage():.2f}V')
    print(f'\tCurrent: {bot.getCurrent():.2f}A', end="\n\n")

    # Start all the appropriate timers
    StartRun();

    while(not BurgCode()): # Dies when BurgCode returns false
        print(f"\t time: {time.time() - start_time}", end="\n\n");

    ShowStats();