# M4 - Autonomous fruit searching

# basic python packages
import sys, os
import cv2
import numpy as np
import json
import ast
import argparse
import time

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf1 import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco
import matplotlib as plt
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
                    aruco_true_pos[marker_id][0] = x
                    aruco_true_pos[marker_id][1] = y
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
    dir = delta_rot - pose_angle
    #anticlk = 360 - clk

    if dir > 0 :
        delta_rot = dir 
        turn_direc = [-turn_vel, turn_vel]

    elif dir < 0:
        delta_rot = -dir
        turn_direc = [turn_vel, -turn_vel]
    else:
        turn_direc = [0, 0]

    # calculate time it takes for a full 360 spin
    full_turn_time = (baseline * np.pi) / (scale * turn_vel) 

    # est turn_time from baseline (requires good calibration)
    turn_time = full_turn_time * (delta_rot/360)

    # potnetial other method
    # turn angle = [0,0,0,0,0,...]
    # Extrapolate turn time based on estimated time it takes to make 45, 90, 135, 180, 215, 275, 315, 360 deg rotation
    # Use if else statement to find condition whereby delta_rot is closest to above values and use formula in method 1

    return turn_direc, turn_time

# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# additional improvements:
# you may use different motion model parameters for robot driving on its own or driving while pushing a fruit
# try changing to a fully automatic delivery approach: develop a path-finding algorithm that produces the waypoints
def drive_to_point(waypoint, robot_pose):
    # imports camera / wheel calibration parameters 
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')
    
    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point

    wheel_vel = [23.6, 25] # tick to move the robot
    turn_vel = 10

    # get dist to waypoint
    delta_x = (waypoint[0][0] - robot_pose[0][0])
    delta_y = (waypoint[0][1] - robot_pose[0][1])
    delta_dist = np.sqrt( delta_x**2  +  delta_y**2 )

    # get angle to waypoint
    delta_rot = np.arctan2(delta_y, delta_x) * 180 / np.pi

    # get drive time (forward)
    drive_time = delta_dist / (scale * wheel_vel[1])

    # get turn direction and time to face waypoint direction
    turn_direc1, turn_time1 = turn_to_point(delta_rot, robot_pose[0][2], turn_vel, scale, baseline)
    
    # turn towards the waypoint
    print("Turning for {} seconds speed {}".format(turn_time1, turn_direc1))
    move_1 = ppi.set_velocity([0, 1], tick=turn_direc1, time=turn_time1)
    img_1 = ppi.get_image()

    # after turning, drive straight to the waypoint
    print("Driving for {} seconds speed {}".format(drive_time, wheel_vel))
    move_2 = ppi.set_velocity([1, 0], tick=wheel_vel, time=drive_time)
    img_2 = ppi.get_image()

    # turn direction and time to match angle of waypoint
    turn_direc2, turn_time2 = turn_to_point(waypoint[0][2],delta_rot,turn_vel,scale,baseline)

    # turn towards the waypoint
    print("Turning for {} seconds speed {}".format(turn_time2, turn_direc2))
    move_3 = ppi.set_velocity([0, 1], tick=turn_direc2, time=turn_time2)
    img_3 = ppi.get_image()
    ####################################################

    ppi.set_velocity([0, 0], tick=[0,0], time=0)
    #time.sleep(5)
    done = 1

    return move_1, move_2, move_3, img_1, img_2, img_3, done
    
def get_robot_pose(move_1, move_2, move_3, img_1, img_2, img_3):
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here

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

    if len(move_3) > 0:
        lv, rv, moving_time = move_3
        drive_meas = measure.Drive(lv, rv, moving_time)

        lms, aruco_img = aruco_det.detect_marker_positions(img_3)
        ekf.predict(drive_meas)
        ekf.update(lms)

    robot_pose = ekf.robot.state
    ####################################################

    return robot_pose

# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='M4_true_map.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    args, _ = parser.parse_known_args()

    ppi = Alphabot(args.ip,args.port)

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list()

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
    meausrements = []
    # intialize ekf, slam and robot
    robot = Robot(baseline, scale, camera_matrix, dist_coeffs)
    ekf = EKF(robot)
    aruco_det = aruco.aruco_detector(ekf.robot, marker_length = 0.06)

    #initialize variables
    x, y, theta = 0.0, 0.0, 0.0
    waypoint = [[0.0, 0.0, 0.0]]
    robot_pose = [[0.0, 0.0, 0.0]]
    move_1 = np.zeros((0,3))
    move_2 = np.zeros((0,3))
    move_3 = np.zeros((0,3))
    img_1 = np.zeros([480, 640, 3], dtype=np.uint8)
    img_2 = np.zeros([480, 640, 3], dtype=np.uint8)
    img_3 = np.zeros([480, 640, 3], dtype=np.uint8)

    # The following code is only a skeleton code the semi-auto fruit searching task
    while True:
        # enter the waypoints
        # instead of manually enter waypoints in command line, you can get coordinates by clicking on a map (GUI input), see camera_calibration.py
        x = input("X coordinate of the waypoint (m): ")
        try:
            x = float(x)
        except ValueError:
            print("Please enter a number.")
            continue
        y = input("Y coordinate of the waypoint (m): ")
        try:
            y = float(y)
        except ValueError:
            print("Please enter a number.")
            continue
        theta = input("Angle theta of the waypoint (deg): ")
        try:
            theta = float(theta)
        except ValueError:
            print("Please enter a number.")
            continue
        
        done = 0
        # robot drives to the waypoint
        # theta = theta * np.pi/180 # conver deg to rad
        while done == 0:
            waypoint = [[x, y, theta]]
            move_1, move_2, move_3, img_1, img_2, img_3, done = drive_to_point(waypoint,robot_pose)

        # estimate the robot's pose
        ppi.set_velocity([0, 0], tick=[0,0], time=0)
        time.sleep(1)
        robot_pose = get_robot_pose(move_1, move_2, move_3, img_1, img_2, img_3)
        robot_pose = robot_pose.T

        print("Finished driving to waypoint: {}; New robot pose: {}".format(waypoint, robot_pose))

        # exit
        ppi.set_velocity([0, 0], tick=[0,0], time=0)
        uInput = input("Add a new waypoint? [y/n]")
        if uInput == 'n':
            break