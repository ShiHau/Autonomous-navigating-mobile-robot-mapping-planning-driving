# M4 - Autonomous fruit searching

# basic python packages
import sys, os
import cv2
import numpy as np
import json
import ast
import argparse
import time
from Astar import AStarPlanner
sys.path.insert(0,"{}/network/".format(os.getcwd()))
sys.path.insert(0,"{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector
import TargetPoseEst as TPE

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf1 import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import utility functions
#sys.path.insert(0, "util")
from util.pibot import Alphabot
import util.measure as measure


def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 3 types of target fruit to search

    @param fname: filename of the map
    @return:
        1) list of target fruits, e.g. ['redapple', 'greenapple', 'orange']
        2) locations of the target fruits, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    with open(fname, 'r') as f:
        try:
            gt_dict = json.load(f)                   
        except ValueError as e:
            with open(fname, 'r') as f:
                gt_dict = ast.literal_eval(f.readline())   
        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = np.empty([10, 2])

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                else:
                    marker_id = int(key[5])
                    aruco_true_pos[marker_id-1][0] = x
                    aruco_true_pos[marker_id-1][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        return fruit_list, fruit_true_pos, aruco_true_pos


def read_search_list():
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('search_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(3):
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
        n_fruit += 1

def turn_to_point(delta_rot, pose_angle, turn_vel, scale, baseline):
    # find optimal turn direction
    dir = delta_rot - pose_angle*180/np.pi

    #print("dir: ",dir)

    if dir > 0 :
        turn_direc = [-turn_vel, turn_vel]

    elif dir < 0:
        turn_direc = [turn_vel, -turn_vel]
        # offset for right turn beecause it is shorter than left turn 
        ppi.set_velocity([0,1],turn_direc,time=0.15)
    else:
        turn_direc = [0, 0]

    # calculate time it takes for a full 360 spin
    full_turn_time = (baseline * np.pi) / (scale * turn_vel) 

    # est turn_time from baseline (requires good calibration)
    turn_time = full_turn_time * (abs(dir)/360)
    if dir == 0:
        turn_time = 0

    return turn_direc, turn_time

# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
def drive_to_point(scale, baseline, waypoint, robot_pose):
    move_1, move_2 = [0,0,0], [0,0,0]
    wheel_vel = [23.6, 25] # tick to move the robot
    turn_vel = 10

    # get dist to waypoint
    delta_x = (waypoint[0][0] - robot_pose[0][0])
    delta_y = (waypoint[0][1] - robot_pose[0][1])
    delta_dist = np.sqrt( delta_x**2  +  delta_y**2 )

    # get angle to waypoint
    delta_rot = np.arctan2(delta_y, delta_x) * 180 / np.pi
    #print("delta rot: ",delta_rot)
    if delta_x == 0 and delta_y == 0:
        delta_rot = 360

    # limit delta_rot between 0 and 360 deg
    delta_rot = delta_rot % 360
    if delta_rot == 360:
        delta_rot = 0

    # get drive time (forward)
    drive_time = delta_dist / (scale * wheel_vel[1])

    # get turn direction and time to face waypoint direction
    turn_direc1, turn_time1 = turn_to_point(delta_rot, robot_pose[0][2], turn_vel, scale, baseline)
    
    # turn towards the waypoint
    if turn_time1 > 0.01:
        print("Turning for {} seconds speed {}".format(turn_time1, turn_direc1))
        move_1 = ppi.set_velocity([0, 1], tick=turn_direc1, time=turn_time1)
    img_1 = ppi.get_image()

    DL, DR = ppi.get_IR() # stop robot from driving forward if detect object after turning
    if DL == 0 or DR == 0:
        drive_time = 0

    if drive_time > 0.01:
        # after turning, drive straight to the waypoint
        print("Driving for {} seconds speed {}".format(drive_time, wheel_vel))
        move_2 = ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    img_2 = ppi.get_image()

    ppi.set_velocity([0,0],tick=[0,0])
    
    return move_1, move_2, img_1, img_2
    
def get_robot_pose(move_1, move_2, img_1, img_2):

    # update the robot pose [x,y,theta]
    robot_pose = [[0.0,0.0,0.0]] # replace with your calculation

    if len(move_1) > 0:
        lv, rv, moving_time = move_1
        drive_meas = measure.Drive(lv, rv, moving_time)
        lms, aruco_img = aruco_det.detect_marker_positions(img_1)
        ekf.predict(drive_meas)
        ekf.update(lms)

    if len(move_2) > 0:
        lv, rv, moving_time = move_2
        drive_meas = measure.Drive(lv, rv, moving_time)
        
        lms, aruco_img = aruco_det.detect_marker_positions(img_2)
        ekf.predict(drive_meas)
        ekf.update(lms)

    robot_pose = ekf.robot.state

    return robot_pose.T

def check_IR(robot_pose):
    # check IR sensor for obstacle nearby
    DL, DR = ppi.get_IR()
    triggered = 0
    if DL == 0 or DR == 0:
        print("IR detected object")
        move_1 = ppi.set_velocity([0,-1],[14.7,15],time=1)
        img_1 = ppi.get_image()
        robot_pose = get_robot_pose(move_1, [0,0,0], img_1, img_1)
        triggered = 1
    # spin, check and update position of robot
    return robot_pose, triggered

def calib_Rob_Pose():
    # to define turn duration for calibration
    full_turn_time = (baseline * np.pi) / (scale * 10)
    turn_time = full_turn_time  * 30/360
    img_1 = ppi.get_image()

    #print("Checking Pose")

    # keep spinning to check position until robot detects at least 2 markers
    lms, aruco_img = aruco_det.detect_marker_positions(img_1)
    while len(lms) < 2:
        move_1 = ppi.set_velocity([0,1],tick=[10,-10],time=turn_time)
        img_1 = ppi.get_image()
        lv, rv, moving_time = move_1
        drive_meas = measure.Drive(lv, rv, moving_time)
        lms, aruco_img = aruco_det.detect_marker_positions(img_1)
        ekf.predict(drive_meas)
        ekf.update(lms)
        time.sleep(1)
        robot_pose = ekf.robot.state.T

    print("Calibrated robot pose: ", robot_pose, waypoint)

    return robot_pose

def path_planning(sx,sy,gx,gy,ox,oy, grid_size, robot_radius, offset):
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    # current output goes to position of fruit directly, need to adjust offset by radius around the fruit
    rx, ry = a_star.planning(sx, sy, gx, gy) # Returns coordinates from goal to start
    # get distance between rx,ry[0] and rx,ry[1]
    rx_dist = rx[0] - rx[1]
    ry_dist = ry[0] - ry[1]
    # find the intersecting point between the two where distance from robot to rx,ry = offset
    # update rx,ry[0] to intersect position
    rx[0] = rx[1] + rx_dist*offset/grid_size
    ry[0] = ry[1] + ry_dist*offset/grid_size
    # update start position for next fruit
    sx = rx[1]
    sy = ry[1]

    return sx, sy, rx, ry

def det_fruits(img, camera_matrix, missing_fruits):
    fx = []
    fy = []
    target_pose_dict = {}
    # get yolo detection
    detector_output, network_vis = detector.yolo_detection(img)

    # get estimated positions of detected fruits
    completed_img_dict = TPE.get_image_info(detector_output, ekf.robot.state)
    target_pose_dict = TPE.estimate_pose(camera_matrix, completed_img_dict)
    for fruit in missing_fruits:
        if fruit in target_pose_dict:
            fx.append(np.round(target_pose_dict[fruit]['x']/0.4)*0.4)
            fy.append(np.round(target_pose_dict[fruit]['y']/0.4)*0.4)
            print("detected,", fruit, " at ", np.round(target_pose_dict[fruit]['x']/0.4)*0.4, np.round(target_pose_dict[fruit]['y']/0.4)*0.4)

    return fx, fy

# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    parser.add_argument("--ckpt", default='network/fruits_model/best.pt')
    args, _ = parser.parse_known_args()

    ppi = Alphabot(args.ip,args.port)
    detector = Detector(args.ckpt, use_gpu=False)

    # read in the true map
    full_fruits_list = ['redapple', 'greenapple', 'orange', 'mango', 'capsicum']
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list()
    missing_fruits = []
    for fruit in full_fruits_list: # list out all unknown fruits positions
        if (fruit in fruits_list) == False:
            missing_fruits.append(fruit)

    fruits_true_pos_seq = [[],[],[]] # number of fruits to drive to and sequence

    # import all param files for Robot()
    datadir = 'calibration/param/'
    fileK = "{}intrinsic.txt".format(datadir)
    camera_matrix = np.loadtxt(fileK, delimiter=',')
    fileD = "{}distCoeffs.txt".format(datadir)
    dist_coeffs = np.loadtxt(fileD, delimiter=',')
    fileS = "{}scale.txt".format(datadir)
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "{}baseline.txt".format(datadir)  
    baseline = np.loadtxt(fileB, delimiter=',')

    # intialize ekf, slam and robot
    robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
    ekf = EKF(robot)
    aruco_det = aruco.aruco_detector(ekf.robot, marker_length = 0.06)

    #initialize variables
    x, y = 0.0, 0.0
    sx, sy = 0.0, 0.0
    waypoint = [[0.0, 0.0, 0.0]]
    robot_pose = [[sx, sy, 0.0]]
    move_1 = np.empty((0,3))
    move_2 = np.empty((0,3))
    img_1 = np.zeros([480, 640, 3], dtype=np.uint8)
    img_2 = np.zeros([480, 640, 3], dtype=np.uint8)

    grid_size, robot_radius, offset = 0.2, 0.12, 0.0  # [m]
    pose_error = 0.2

    # obstacle coordinates
    ox, oy = [], []

    # arrange search order sequence
    for i in range(0,len(search_list)):
        for j in range(0,len(fruits_list)):
            if search_list[i] == fruits_list[j]:
                fruits_true_pos_seq[i] = fruits_true_pos[j]

    # store all known obstacle positions in ox and oy
    for marker in range(0,len(aruco_true_pos)):
        ox.append(aruco_true_pos[marker][0])
        oy.append(aruco_true_pos[marker][1])

    # check initial postion of robot(in case robot wasn't placed perfectly at 0,0,0)
    img_1 = ppi.get_image()
    robot_pose = get_robot_pose([0,0,0],[0,0,0],img_1,img_1)
    sx, sy = robot_pose[0][0], robot_pose[0][1]

    while len(fruits_true_pos_seq) != 0: # while still have fruit to drive to

        ppi.set_velocity([0, 0], tick=[0,0])
        time.sleep(1)
        
        gx = fruits_true_pos_seq[0][0]
        gy = fruits_true_pos_seq[0][1]

        sx,sy,rx,ry = path_planning(sx,sy,gx,gy,ox,oy,grid_size,robot_radius,offset)

        print("path")
        print(rx)
        print(ry)
        print("Start drive")

        for j in range(len(rx)-2,-1,-1): # move robot to specified waypoint
            waypoint[0][0] = rx[j]
            waypoint[0][1] = ry[j]

            if [waypoint[0][0],waypoint[0][1]] == [rx[0],ry[0]] or j == 1:
                move_1, move_2, img_1, img_2 = drive_to_point(scale, baseline, waypoint,robot_pose)
                robot_pose = get_robot_pose(move_1, move_2, img_1, img_2)
                print(j, " robot pose: ", robot_pose, " waypoint: ",waypoint)

                print("Fruit Reached")
                ppi.set_Buzzer(1)
                ppi.set_velocity([0, 0], tick=[0,0])
                fruits_true_pos_seq.pop(0)
                time.sleep(1)
                ppi.set_Buzzer(0)
                break

            if [waypoint[0][0],waypoint[0][1]] != [rx[0],ry[0]]: # if haven't reached destination
                move_1, move_2, img_1, img_2 = drive_to_point(scale, baseline, waypoint,robot_pose)
                robot_pose = get_robot_pose(move_1, move_2, img_1, img_2)
                print(j, " robot pose: ", robot_pose, " waypoint: ",waypoint)
                ppi.set_velocity([0, 0], tick=[0,0])

                """
                robot_pose, triggered = check_IR(robot_pose)
                if triggered == 1:
                    sx, sy = robot_pose[0][0], robot_pose[0][1]
                    break
                """

                # check if unknown fruits are detected
                fx,fy = det_fruits(img_2, camera_matrix, missing_fruits)
                if len(fx) !=0: # update obstacle list to include position of unknown fruits 
                    for marker in range(0,len(fx)):
                        ox.append(fx[marker])
                        oy.append(fy[marker])
                    sx, sy = robot_pose[0][0], robot_pose[0][1]
                    break # quit for loop to redo path planning and restart travelling

                #robot_pose = calib_Rob_Pose() # check robot position
                # check if position is as expected if not, correct position and replan path
                """
                if waypoint[0][0]-pose_error>robot_pose[0][0] or waypoint[0][0]+pose_error<robot_pose[0][0] or waypoint[0][1]-pose_error>robot_pose[0][1] or waypoint[0][1]+pose_error<robot_pose[0][1]:
                    print("position majorly wrong, replanning path")
                    move_1, move_2, img_1, img_2 = drive_to_point(scale, baseline, waypoint,robot_pose)
                    robot_pose = get_robot_pose(move_1, move_2, img_1, img_2)
                    sx = waypoint[0][0]
                    sy = waypoint[0][1]
                    break
                """
                
            ppi.set_velocity([0, 0], tick=[0,0])
            time.sleep(1)
                
    ppi.set_velocity([0, 0], tick=[0,0])
        