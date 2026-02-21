import cv2
from Rosmaster_Lib import Rosmaster


def init_robot():
    """
    Initialize the ROSMaster R2 robot and start its receive thread.
    """
    bot = Rosmaster()
    bot.create_receive_threading()
    print("Robot initialized.")
    return bot


def init_camera(camera_index: int = 0):
    """
    Initialize the camera capture.
    """
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("Error: Could not open camera.")
    print("Camera initialized on index", camera_index)
    return cap


def stop_robot(bot):
    """
    Stop all robot motors.
    """
    if bot:
        bot.set_motor(0, 0, 0, 0)
        print("Robot motors stopped.")
