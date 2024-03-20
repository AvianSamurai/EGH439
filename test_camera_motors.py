import argparse
import time
import cv2
from pibot_client import PiBot

# To run this file, run the following command through a terminal (command prompt)
# python test_camera_motors.py --ip <IP>
# where <IP> has been replaced with the IP address of your robot.
#
# If you are still having issues, it could be due to the issues of getting windows
# to recognise conda. Instead, go to line 20 and replace "ip=args.ip" with the
# IP address of your robot, and run this file by paly button in the top right
# if using VS code. Note that the IP address needs to be enclosed in quotation
# marks to make it a string.
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='PiBot client')
    parser.add_argument('--ip', type=str, default='192.168.50.5', help='IP address of PiBot')
    args = parser.parse_args()

    bot = PiBot(ip=args.ip)

    print(f'Voltage: {bot.getVoltage():.2f}V')
    print(f'Current: {bot.getCurrent():.2f}A')

    enc_begin_left, enc_begin_right = bot.getEncoders()
    print(f"get encoders state at beginning: {enc_begin_left}, {enc_begin_right}")

    print("speed test");
    time.sleep(5);
    bot.setVelocity(100, 100, 1);
    time.sleep(60);

    print("test left motor")
    bot.setVelocity(10,0)
    time.sleep(2)

    print("test right motor")
    bot.setVelocity(0,10)
    time.sleep(2)

    print("stop")
    bot.setVelocity(0,0)

    enc_end_left, enc_end_right = bot.getEncoders()
    print(f"get encoders state at end: {enc_end_left}, {enc_end_right}")

    print("initialise camera")
    time.sleep(2)
    print("grab image")
    image = bot.getImage()
    print(f"image size {image.shape[0]} by {image.shape[1]}")

    try:
        while True:
            cv2.imshow('image', image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            image = bot.getImage()
    except KeyboardInterrupt:
        exit()

