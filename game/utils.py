import airsim
from airsim.types import Pose, Vector3r, Quaternionr
import time
import numpy as np

def init_sim():
    # connect to the AirSim simulator
    drone = airsim.MultirotorClient()
    drone.confirmConnection()
    drone.reset()
    time.sleep(1)

    if drone.isApiControlEnabled():
        drone.armDisarm(False)
        set_pose(drone,0,0,0,0,0,0)
        
    drone.enableApiControl(True)
    drone.armDisarm(True)
    return drone

def takeoff(drone:airsim.MultirotorClient):
    drone.takeoffAsync().join()

def land(drone:airsim.MultirotorClient):
    drone.landAsync().join()

def turn_off_drone(drone:airsim.MultirotorClient):
    drone.armDisarm(False)
    drone.enableApiControl(False)

####################################################################################################################################
### Drone Pose functions

def get_position(x: float = 0,y: float = 0, z: float = 0):
    return Vector3r(x,y,z)

def get_orientation(roll: float = 0, pitch: float = 0, yaw: float = 0):
    return airsim.to_quaternion(pitch,roll,yaw)

def get_drone_position(drone: airsim.MultirotorClient):
    return drone.simGetVehiclePose().position


def set_pose(drone: airsim.MultirotorClient, x=0,y=0,z=0, roll=0,pitch=0,yaw=0):
    goal_pos = get_position(x,y,z)
    while distance_to_goal(drone, goal_pos, True) > 1:
        drone.simSetVehiclePose(Pose(goal_pos, get_orientation(roll,pitch,yaw)), True)
    time.sleep(1)

def teleport_drone(location='start'):
    locations = {
        'start': Pose(get_position(0,0,0), get_orientation(0,0,0)),
        'easy_maze': Pose(get_position(0,-68,0), get_orientation(0,0,0))
    }
    if not location in locations:
        location = 'start'

    return locations[location]    


def distance_to_goal(drone: airsim.MultirotorClient, goal: Vector3r, scalar=True) -> Vector3r:
    position = drone.simGetVehiclePose().position
    if scalar:
        return position.distance_to(goal)
    else:
        return goal - position
    
####################################################################################################################################


####################################################################################################################################
## Movement Functions

def moveByVelocity(drone: airsim.MultirotorClient, desired_velocity: Vector3r, maxVelocity: float= 5.0, waitToFinish: bool=False):
    speed = desired_velocity.get_length()
    if speed > maxVelocity:
        desired_velocity *= (maxVelocity/speed)

    func = drone.moveByVelocityAsync(desired_velocity.x_val, desired_velocity.y_val, desired_velocity.z_val, 5)
    if waitToFinish:
        func.join()
    pass


def moveByBodyVelocity(drone: airsim.MultirotorClient, vx:float=0, vy:float=0, vz:float=0, duration:float=1, waitToFinish: bool=True):
    func = drone.moveByVelocityBodyFrameAsync(vx,vy, vz,duration)
    if waitToFinish:
        func.join()
    pass

def moveByVelocityAndZ(drone: airsim.MultirotorClient, desired_velocity: Vector3r, maxVelocity: float= 5.0, waitToFinish: bool=False):
    speed = desired_velocity.get_length()
    if speed > maxVelocity:
        desired_velocity *= (maxVelocity/speed)

    func = drone.moveByVelocityZAsync(desired_velocity.x_val, desired_velocity.y_val, -desired_velocity.z_val, 5)
    if waitToFinish:
        func.join()
    pass

def moveByRPYAndZ(drone: airsim.MultirotorClient, roll:float, pitch:float, yaw:float, z:float, duration=1, waitToFinish: bool=False):
    func = drone.moveByRollPitchYawZAsync(roll, pitch, yaw, z, duration)
    if waitToFinish:
        func.join()
    pass


def moveToPosition(drone: airsim.MultirotorClient, goal: Vector3r, speed: float= 5.0, waitToFinish: bool=True):
    func = drone.moveToPositionAsync(goal.x_val, goal.y_val, goal.z_val, speed)
    if waitToFinish: 
        func.join()

def move_to_altitude(drone: airsim.MultirotorClient, altitude: float, speed=1) -> None:
    drone.moveToZAsync(altitude,speed).join()

def look_in_direction_angle(drone: airsim.MultirotorClient, angle: float) -> None:
    drone.rotateToYawAsync(angle).join()

####################################################################################################################################


####################################################################################################################################
########## WEEK 3 UTILS ############################33

def turn_right(drone: airsim.MultirotorClient, angle:float=90):
    direction, yaw = get_direction_yaw(drone)
    new_angle = ((yaw - angle) % (360)) - 180
    drone.rotateToYawAsync(new_angle).join()


def turn_left(drone: airsim.MultirotorClient, angle:float =90):
    return turn_right(drone, -angle)


def get_week3_win_condition(drone: airsim.MultirotorClient, mode:str):
    if mode == 'easy':
        return drone.simGetVehiclePose().position.x_val < 24.5
    else:
        return drone.simGetVehiclePose().position.y_val < 13.5

def get_direction_yaw(drone:airsim.MultirotorClient, offset:float = 20):
    yaw = airsim.to_eularian_angles(drone.simGetVehiclePose().orientation)[-1] * 180/np.pi
    offset = 20
    if yaw > -90 - offset and yaw < -90 + offset:
        direction = 'left'
        yaw = -90
    elif yaw > 0 - offset and yaw < 0 + offset:
        direction = 'straight'
        yaw = 0
    elif yaw > 90 - offset and yaw < 90 + offset:
        direction = 'right'
        yaw = 90
    else:
        direction = 'back'
        yaw = 180
    return direction, yaw

####################################################################################################################################

def get_image(drone: airsim.MultirotorClient, camera="0", image_type=airsim.ImageType.Scene):
    '''
    Get an image from the Unreal simulator, and form it as a numpy array
    The pixels are [0-??]

    Inputs:
    drone: The drone agent from calling init_sim()
    camera: Which camera do you want? The sim has a few. 0 looks forward. 3 looks down
    image_type: Scene is regular. You may also want DepthVis and Segmentation 
    '''
    responses = drone.simGetImages([airsim.ImageRequest(camera, image_type, False, False)])
    response = responses[0]

    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)

    # original image is fliped vertically
    # img_rgb = np.flipud(img_rgb)
    return img_rgb


def object_detect(drone:airsim.MultirotorClient, camera_name:str="0", image_type=airsim.ImageType.Scene, filter_radius:float=None, mesh_name:str=None):
    '''
    Simulate Computer Vision learning and detect objects in the scene

    Inputs:
    drone: The drone agent from calling init_sim()
    camera: Which camera do you want? The sim has a few. 0 looks forward. 3 looks down
    image_type: Scene is regular. You may also want DepthVis and Segmentation 
    filter_radius: How close does an object need to be for detection?
    mesh_name: Filter to specific objects
    '''
    get_image(drone, camera_name)
    if filter_radius:
        drone.simSetDetectionFilterRadius(camera_name, image_type, filter_radius) # in [cm]
    if mesh_name:
        drone.simAddDetectionFilterMeshName(camera_name, image_type, mesh_name) 
    return drone.simGetDetections(camera_name, image_type)