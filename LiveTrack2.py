#import datetime as dt
#import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pibot_client import PiBot
import argparse

parser = argparse.ArgumentParser(description='Live Tracker')
parser.add_argument('--localizer', type=str, default=1, help='Localizer Number')
parser.add_argument('--IP', type=str, default="172.19.232.146", help='Bot IP Address')
args = parser.parse_args()

IP = args.IP;
LOCALIZER_NUM = args.localizer;

# Create figure for plotting
fig = plt.figure()
xs = []
ys = []

# Connecting to Pibot
print("Connecting to the bot\n")
bot = PiBot(ip=IP, localiser_ip=f'egb439localiser{LOCALIZER_NUM}')

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    # Get robot pose
    pose = bot.getLocalizerPose(5);
    RobX, RobY, = pose[0], pose[1]

    # Add x and y to lists
    xs.append(RobX)
    ys.append(RobY)

    # Limit x and y lists to 20 items
    xs = xs[-5:]
    ys = ys[-5:]

    # Draw x and y lists
    plt.clear()
    plt.plot(xs, ys, 'o')

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Bot position')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()