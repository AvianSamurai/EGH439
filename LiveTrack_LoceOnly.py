#import datetime as dt
#import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import subprocess
from localizer_only import Localizer

LOCALIZER_NUM = 2;

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Connecting to Pibot
print("Connecting to the bot\n")
bot = Localizer(localiser_ip=f'egb439localiser{LOCALIZER_NUM}')

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):
    # Get robot pose
    pose = bot.getLocalizerPose(5);
    RobX, RobY, = pose[0], pose[1]

    # Add x and y to lists
    xs.append(RobX)
    ys.append(RobY)

    # Limit x and y lists to 20 items
    xs = xs[-10:]
    ys = ys[-10:]

    # Draw x and y lists
    ax.clear()
    ax.axis([0, 2, 0, 2])
    ax.plot(xs, ys)
    ax.plot(xs, ys, 'ro')

    # Format plot
    plt.title('Live Robot Tracking')
# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=500)
plt.show()