
# import setup_path
import airsim
from airsim.types import Pose, Vector2r, Vector3r
from airsim.utils import to_eularian_angles

import numpy as np
import time
from utils import *
import cv2


def up_down_left_right_maze():
    '''
    Navigate to the end of the maze!

    QR Codes are placed on the walls to guide you through the maze
    You can read the codes with this: corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict)
    ids gives you an array of detected QR ids.
    Here's what you need to do when you read a marker:
    0: Fly up
    1: Fly down
    2: Fly left
    3: Fly right
    4: Congrats, you're done!
    '''

    # Initialize simulator
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    drone = init_sim()
    takeoff(drone)

    velocity = 3

    # Get an image and check for aruco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(get_image(drone), aruco_dict)

    done_number = -1
    up_number = -1
    down_number = -1
    left_number = -1
    right_number = -1
        
    # Reset these values to the key in the comments above
    # done_number = __
    # up_number = __
    # down_number = __
    # left_number = __
    # right_number = __


    # While we haven't got to the end (or we don't see a marker)
    while ids is None or not ids[0] == done_number:

        # Get a new picture and check for marker
        # corners, ids, rejected = cv2.aruco.___(get_image(drone), aruco_dict)
        position = drone.simGetVehiclePose().position

        # Make sure we have a marker
        if ids == None or len(ids) == 0:
            # Sometimes I was flying too close, and needed to back up to see the marker
            position.x_val -= 2
            moveToPosition(drone, position, 3, True)
            continue

        # Act accordingly
        # Challenge: Can you replace this if statement with a dictionary?
        if ids[0] == up_number: # Move up
            ## Set Y And/Or Z Values Here
            # position.y_val = ___
            # position.z_val = ___
            pass
        elif ids[0] == down_number: # Move down
            ## Set Y And/Or Z Values Here
            pass
        elif ids[0] == left_number: # Move Left
            ## Set Y And/Or Z Values Here
            pass
        elif ids[0] == right_number: # Move right
            ## Set Y And/Or Z Values Here
            pass
        
        # Move to avoid the wall
        moveToPosition(drone, position, velocity, True)

        # Move forward (increase x)
        # position.x_val += ___
        moveToPosition(drone, position, velocity, True)

    land(drone)
    turn_off_drone(drone)


def pineapple_detection():
    '''
    Challenge Activity 2:
    At y=30 there are several objects in the +x direction
    Fly above those objects and land next to the pineapple!
    '''
    # Startup code
    drone = init_sim()
    set_pose(drone, 0, 30, 0)
    takeoff(drone)

    # Fly to a preset location (Play around with this to line up with the objects, or move just to the side)
    position = drone.simGetVehiclePose().position
    # position.z_val = __
    # position.x_val = __
    # position.y_val = __
    moveToPosition(drone,position,waitToFinish=True)

    while True:
        # Detect objects under the drone
        detections = object_detect(drone, "3", mesh_name = '*Pine*', filter_radius=1000)

        # If we find a pineapple, then break the loop
        # Detections is a list. How can we check if it is empty??
        # if ___:
            # break
        
        # If we don't see a pineapple, fly forward. (Pick one, but I think one of them is easier)
        # moveByVelocityAndZ(drone, Vector3r(__,__,__))
        # moveToPosition(drone, Vector3r(__,__,__))



    ## Land the drone and turn it off
    land(drone)
    turn_off_drone(drone)


if __name__=="__main__":
    # up_down_left_right_maze()
    pineapple_detection()

