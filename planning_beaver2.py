
import airsim
import time
from utils import *


def find_wall_and_turn_around(drone:airsim.MultirotorClient):
    '''
    Challenge 1: Make the drone fly back and forth turning at the wall
    '''
    print('You selected: Find wall and turn around')
    velocity = _
    distance_to_wall = 
    fly_time = _

    # Fly back and forth FOREVER!!!!
    while True:
        straight_distance = drone.getDistanceSensorData('DistanceStraight')
        while straight_distance.distance > __:

            # Fly forward for a given time
            moveByBodyVelocity(drone,__,0,0,__)

            # Get a new sensor measurement
            straight_distance = drone.getDistanceSensorData('DistanceStraight')

        # Turn around
        turn_right(drone, 0)



def hard_code_maze(drone:airsim.MultirotorClient):
    '''
    Challenge 2: Solve the easy maze hardcoded
    '''
    print('You selected: Hard Code Maze')
    
    velocity = _
    distance_to_wall = 
    fly_time = _

    # Move forward and turn left
    while drone.__.distance > __:
        moveByBodyVelocity(___)
    turn____(drone)

    # Move forward and turn right


    # Move forward until wall

    

def my_fly_function(drone:airsim.MultirotorClient, velocity:float, fly_time:float, wall_distance:float, turn:str=''):
    '''
    Challenge 3.5: Write a function to fly until you get to a wall, then turn

    Inputs:
    velocity - How fast the drone flies
    fly_time - How long to fly
    distance - How close to the wall to stop flying
    turn     - Which direction to turn after stopping
    '''
    ## Move forward until wall
    while __ > __:
        __

    # Use the turn_left and turn_right functions here
    if ___:
        __(drone)
    elif __ == __:
        __(drone)
    

def hard_code_maze_with_function(drone:airsim.MultirotorClient):
    '''
    Challenge 3: Simplify Challenge 2 using functions!
    '''
    print('You selected: Hard Code Maze with Function')
    velocity = __
    fly_time = __
    distance = __

    # Use the my fly function to simplify Challenge 2
    my_fly_function(drone, ___, ___, ___, ___)
    my_fly_function(__)
    my_fly_function(__)


def solve_maze(drone: airsim.MultirotorClient):
    '''
    Bonus Challenge: Solve the maze without hardcoding values
    '''
    print('You selected: Solve Maze')

    # Find out how wide the corridors are
    left = drone.getDistanceSensorData('DistanceLeft')
    right = drone.getDistanceSensorData('DistanceRight')
    space_distance = left.distance + right.distance

    # Set fly speed
    velocity = __
    
    while get_week3_win_condition(drone, mode):

        # Get Sensor Readings
        straight = drone.__
        left = drone.__
        right = drone.__

        # What should we do if nothing is to our right?
        if right.__ > 0.75 * space_distance:
            moveByBodyVelocity(drone,0,0,0,1) # Pause
            turn___(drone)

            # If we can, move straight
            if drone.___.distance > ___:
                ___

        # What should we do if nothing is in front of us?
        elif ___.distance > ___:
            moveByBodyVelocity(___)
            moveByBodyVelocity(drone,0,0,0,1)

        else: # What should we do if nothing else?
            moveByBodyVelocity(drone,0,0,0,1)
            turn___(drone)
            
            # If we can, move straight
            if drone.___.distance > ___:
                ___


if __name__=="__main__":

    drone = init_sim()

    mode = 'easy'
    if mode == 'easy':
        set_pose(drone, y=-68)

    takeoff(drone)
    time.sleep(1)

    drone.moveToZAsync(-2,1).join()

    find_wall_and_turn_around(drone)
    # hard_code_maze(drone)
    # hard_code_maze_with_function(drone)
    # solve_maze(drone)

    land(drone)
    turn_off_drone(drone)