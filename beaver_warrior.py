
# import setup_path
import airsim
from airsim.types import Pose, Vector2r, Vector3r

import numpy as np
import time
from utils import *
import cv2


def get_function_and_condition(mode:str):
    vals = {
        'hoop_race': [hoop_race, hoop_race_win],
        'distance_maze': [distance_maze, distance_maze_win],
        'vision_maze': [vision_maze, vision_maze_win],
        'sar_map': [sar_map, sar_map_win],
        'fake_walls': [fake_walls, fake_walls_win],
    }
    return vals[mode][0], vals[mode][1]


def hoop_race(drone:airsim.MultirotorClient):
    set_pose(drone, 8, 15, yaw=np.pi/2)
    velocity = 20
    moveByBodyVelocity(drone, velocity, 0, 0, 2)
    #print(drone.simGetVehiclePose().position.y_val)   



def distance_maze(drone:airsim.MultirotorClient):
    # Find out how wide the corridors are
    left = drone.getDistanceSensorData('DistanceLeft')
    right = drone.getDistanceSensorData('DistanceRight')
    space_distance = left.distance + right.distance

    # Set fly speed
    velocity = 4
    target = 6
    desired_right_distance = 1.3  # Desired distance from the right wall

    while get_week3_win_condition(drone, mode):
    #while 1:
        # Get Sensor Readings
        straight = drone.getDistanceSensorData('DistanceStraight')
        left = drone.getDistanceSensorData('DistanceLeft')
        right = drone.getDistanceSensorData('DistanceRight')
        
        print(f'y {drone.simGetVehiclePose().position.y_val}')
        print(f'x {drone.simGetVehiclePose().position.x_val}')
        #around goal now. go in. 
        if drone.simGetVehiclePose().position.y_val >6 and drone.simGetVehiclePose().position.x_val <11:
            turn_left(drone)
            moveByBodyVelocity(drone, velocity, 0, 0, 8)

        if straight.distance >= target:
            # Adjust the drone's position to maintain the desired distance from the right wall
            if right.distance > desired_right_distance:
                moveByBodyVelocity(drone, velocity, 0.5, 0, 1)  # Move slightly to the right
            elif right.distance < desired_right_distance:
                moveByBodyVelocity(drone, velocity, -0.5, 0, 1)  # Move slightly to the left
            else:
                moveByBodyVelocity(drone, velocity, 0, 0, 1.5)  # Move straight forward

        # If there's not enough space in front, turn right
        elif right.distance > target:
            turn_right(drone)
            moveByBodyVelocity(drone, velocity, 0, 0.5, 1)

        # If there's no space in front or to the right, turn left
        else:
            turn_left(drone)
            moveByBodyVelocity(drone, velocity, 0, 0, 1)
        

def vision_maze(drone:airsim.MultirotorClient):
    # When drone sees Aruco 4
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
    #drone = init_sim()
    #takeoff(drone)
    print("taking off")

    velocity = 4

    # Get an image and check for aruco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(get_image(drone), aruco_dict)

    done_number = 4
    up_number = 0
    down_number = 1
    left_number = 2
    right_number = 3
    
    position = drone.simGetVehiclePose().position
    position.z_value = 6000
    moveToPosition(drone, position, 1, True)

    # While we haven't got to the end (or we don't see a marker)
    while ids is None or not ids[0] == done_number:

        # Get a new picture and check for ma0rker
        corners, ids, rejected = cv2.aruco.detectMarkers(get_image(drone), aruco_dict)
        position = drone.simGetVehiclePose().position

        print(f'y {position.y_val} x {position.x_val} z {position.z_val} ids {ids}')
        # Make sure we have a marker
        if ids == None or len(ids) == 0:
            # Sometimes I was flying too close, and needed to back up to see the marker
            position.x_val -= 1.5
            #position.z_val -= 1
            position.y_val -= 0.7
            moveToPosition(drone, position, 1, True)
            continue

        # Act accordingly
        # Challenge: Can you replace this if statement with a dictionary?
        if ids[0] == up_number: # Move up
            ## Set Y And/Or Z Values Here
            #position.y_val = ___
            position.z_val -= 10
            #pass
        elif ids[0] == down_number: # Move down
            ## Set Y And/Or Z Values Here
            position.z_val += 10
            #pass
        elif ids[0] == left_number: # Move Left
            ## Set Y And/Or Z Values Here
            position.y_val -= 7
            #pass
        elif ids[0] == right_number: # Move right
            ## Set Y And/Or Z Values Here
            position.y_val += 10
            #pass
        
        # Move to avoid the wall
        moveToPosition(drone, position, velocity, True)

        # Move forward (increase x)
        position.x_val += 15
        moveToPosition(drone, position, velocity, True)

    land(drone)
    turn_off_drone(drone)

def find_wall_and_turn_around(drone:airsim.MultirotorClient):
    '''
    Challenge 1: Make the drone fly back and forth turning at the wall
    '''
    print('You selected: Find wall and turn around')
    velocity = 2
    while True:
        straight = drone.getDistanceSensorData('DistanceStraight')
        while straight.distance > 5:
            moveByBodyVelocity(drone,velocity,0,0,0.5)
            straight = drone.getDistanceSensorData('DistanceStraight')
        turn_right(drone, 0)

def sar_map(drone:airsim.MultirotorClient):
    import time
    import random
    sar_setup_map(drone, 15)

    detections = object_detect(drone, "3", mesh_name = '*Target*', filter_radius=1000)
    print(f'1:{detections}')
    # Fly to a preset location (Play around with this to line up with the objects, or move just to the side)
    position = drone.simGetVehiclePose().position
    print(f'y {position.y_val} x {position.x_val} z {position.z_val} ')
    position.x_val = 0
 
    moveToPosition(drone,position,3, True)
    print("2")
    time.sleep(1)

    position = drone.simGetVehiclePose().position
    print(f'y {position.y_val} x {position.x_val} z {position.z_val} ')    
    
    detections = object_detect(drone, "3", mesh_name = '*Target*', filter_radius=1000)
    print(detections)


    while True:
        # Detect objects under the drone
        detections = object_detect(drone, "3", mesh_name = '*Target*', filter_radius=1000)
        print(detections)
        print(f'y {position.y_val} x {position.x_val} z {position.z_val} ')
        # If we find a pineapple, then break the loop
        # Detections is a list. How can we check if it is empty??
        if 'Target' in detections:
            print("Found a Target!")
            break
        
        # If we don't see a pineapple, fly forward. (Pick one, but I think one of them is easier)
        #moveByVelocityAndZ(drone, Vector3r(__,__,__))
        #generate two random numbers between 0 and 30
        z = position.x_val+random.randint(0,5)
        y = position.y_val+ random.randint(0,5)
        x = 100
    
        moveByVelocityAndZ(drone, Vector3r(x,y,z))
        #moveToPosition(drone,position,3, True)
        time.sleep(2)
        if  drone.getDistanceSensorData('DistanceStraight').distance < 10:
            turn_right(drone, 0)
  



def fake_walls(drone:airsim.MultirotorClient):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    corners, ids, rejected = cv2.aruco.detectMarkers(get_image(drone), aruco_dict)
    pass


if __name__=="__main__":
    drone = init_sim()
    
    # Start Timer
    start_time = time.perf_counter()

    ############################################################
    ############## Run Function ################################
    # 'hoop_race', 'distance_maze', 'vision_maze', 'sar_map', 'fake_map'
    #mode = 'distance_maze'
    mode = 'sar_map'
    #mode = 'vision_maze'
    #mode = 'hoop_race'
    ############################################################
    takeoff(drone)
    challenge_func, win_func = get_function_and_condition(mode)
    challenge_func(drone)
    results = win_func(drone)
    print(results)


    # End Timer
    end_time = time.perf_counter()
    duration = end_time - start_time
    print(f'Total Time: {end_time - start_time}')

    record_run(mode, duration, results)