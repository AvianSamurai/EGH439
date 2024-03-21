import time
import numpy as np
import matplotlib.pyplot as plt

#from pibot_client import PiBot
from localizer_only import Localizer
# [===============================[ SETTINGS ]==================================]
# Connection settings
IP = "172.19.232.146"
USE_LOCALIZER = True;
LOCALIZER_NUM = 2;
print("Connecting to the bot\n")
#bot = PiBot(ip=IP, localiser_ip=f'egb439localiser{LOCALIZER_NUM}')
bot = Localizer(localiser_ip=f'egb439localiser{LOCALIZER_NUM}')
CurrPose = [0, 0, 0]
i = 0

#PoseInit = bot.getLocalizerPose(9)
PrevEncode = np.array([0, 0])
#PrevX = PoseInit[0]
#PrevY = PoseInit[1]
#PrevTheta = PoseInit[2]
VehicleWidth = 150/1000
WheelRadius = 65.7/2/1000
EncoderTickDis = (WheelRadius * 2 * np.pi)/377
Transmis = np.array([EncoderTickDis, EncoderTickDis]) # encoder to displacement factor [L, R]

#subprocess.Popen(f"cmd /c python LiveTrack_LoceOnly.py", cwd=os.getcwd(), shell=True);

StartTime = time.time()
LastTime = StartTime

def LoceSpeed():
    global i, CurrPose, StartTime, PrevEncode, PrevTheta, VehicleWidth, WheelRadius, LastTime, Transmis
    
    Pose = bot.getLocalizerPose(9)
    #Encode = np.asarray(bot.getEncoders())
    Encode = np.array(np.array([377, 377]) * i)
    
    i = i + 1
    if Pose == CurrPose:
        loceOutputState = 'Dupe'
        PrevX = Pose[0]
        PrevY = Pose[1]
        PrevTheta = Pose[2]

        TimeDiff = time.time() - LastTime
        EncoderDiff = Encode - PrevEncode
        EncoderDisp = EncoderDiff * Transmis
        WheelVel = EncoderDisp / TimeDiff
        TVelocity = 0.5 * (WheelVel[0] + WheelVel[1])
        
        GuessX = PrevX + TVelocity * np.cos(np.deg2rad(PrevTheta))
        GuessY = PrevY + TVelocity * np.sin(np.deg2rad(PrevTheta))
        GuessTheta = PrevTheta + (((WheelVel[0] - WheelVel[1]) / VehicleWidth) * TimeDiff)
        
        Pose = [GuessX, GuessY, GuessTheta]
        CurrPose = Pose
    elif not Pose == CurrPose:
        loceOutputState = 'New'
        CurrPose = Pose
    else:
        loceOutputState = 'Error'
        Pose = [0, 0, 0]
    
    PrevEncode = Encode
    PrevTheta = Pose[2]
    CurrTime = time.time() - StartTime
    LastTime = time.time()
    return(i, CurrTime, loceOutputState, Pose)

while (time.time() - StartTime) < 30:
    output = LoceSpeed()
    print(f"{output}")
    time.sleep(0.1)