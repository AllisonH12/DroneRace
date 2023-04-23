import numpy as np
import airsim
import cv2
from utils import *

def setup_map(drone:airsim.MultirotorClient, scale:float):
    '''
    DON'T TOUCH THIS... THIS IS FOR ME TO MOVE THE TARGET
    '''

    drone.simSetObjectPose('Wall1', Pose(get_position(-scale, 0, 0), get_orientation(yaw=np.pi/2)))
    drone.simSetObjectPose('Wall2', Pose(get_position(scale, 0, 0), get_orientation(yaw=np.pi/2)))
    drone.simSetObjectPose('Wall3', Pose(get_position(0,-scale,0), get_orientation()))
    drone.simSetObjectPose('Wall4', Pose(get_position(0,scale,0), get_orientation()))

    tpos = np.random.uniform(-scale+5, scale-5,2)
    drone.simSetObjectPose('Target', Pose(get_position(tpos[0],tpos[1],1), get_orientation(pitch = np.pi/2)))
    # drone.simSetObjectPose('Target', Pose(get_position(0,0,1), get_orientation(pitch = np.pi/2)))
    drone.simSetObjectScale('Target', Vector3r(2,20,20))


def process_detection(drone: airsim.MultirotorClient, detect: list[airsim.DetectionInfo]):
        '''
        This function is the landing sequence. Currently, we just stop where we see the target.
        Can you make it so we actually land on the target?
        '''
        # Move to when the target was discovered
        position = get_drone_position(drone)
        moveToPosition(drone, position, 1)

        # Stretch goal, can you move directly onto the target?
        # max_target_position = detect.box3D.max
        # min_target_position = detect.box3D.min

def lawn_mower(drone: airsim.MultirotorClient, xmin: float, xmax:float, ymin:float, ymax:float, height:float, step:float, velocity:float=2):
    '''
    Fly the drone in a lawnmower pattern until you find the target. 
    You should start at  xmin, ymin and start flying towards xmax (increase x).
    When you hit xmax, move to the right (+y) by step.
    Fly to xmin.
    When you hit xmin, move to the right (+y) by step.
    Repeat until you find the target

    /params:
    drone: The drones
    xmin: The min x value for the search
    ymax: The max x value for the search
    xmin: The min y value for the search
    ymax: The max y value for the search
    height: drone flight altitude
    step: How far to move over in each lawn_mower row
    velocity: How fast to fly

    Fun extra: There is one chunk of code repeated in this function, can
    you rewrite it so you don't repeat it.
    '''
    # Go to starting location (xmin, ymin)
    # moveToPosition(drone, position(x,y,z), how fast?)
    moveToPosition(drone, Vector3r(None, None, None), None)

    # Write a while loop until you hit ymax
    while get_drone_position(drone).y_val < None:

        # Fly to x min
        while __:
            moveByVelocity(drone, Vector3r(None,0,0), None,)
            detects = object_detect(drone, "3", mesh_name = 'Target')

            # Start landing sequence if you detect the target
            if len(detects) > 0:
                return process_detection(drone, detects)
            time.sleep(0.05)

        # Slide over
        position = get_drone_position(drone)

        # Increase the y value in position by step

        # Move to the new position
        # moveToPosition(drone, where to move, how fast)


        # Fly to x min
        while __:
            moveByVelocity(drone, Vector3r(None,0,0), None,)
            detects = object_detect(drone, "3", mesh_name = 'Target')

            # Start landing sequence if you detect the target
            if len(detects) > 0:
                return process_detection(drone, detects)
            time.sleep(0.05)

        # Slide over
        position = get_drone_position(drone)

        # Increase the y value in position by step

        # Move to the new position
        # moveToPosition(drone, where to move, how fast)

if __name__ == "__main__":
    drone = init_sim()

    scale = 10
    takeoff(drone)

    drone.moveToZAsync(-1,3).join()
    setup_map(drone, scale)
    scale -= 3
    lawn_mower(drone, -scale, scale, -scale, scale, -1, 4, 2)

    land(drone)
    turn_off_drone(drone)
    pass