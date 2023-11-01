# M4 - Autonomous fruit searching

# basic python packages
import sys, os
import numpy as np
import json
import ast
import argparse
import time
import matplotlib.pyplot as plt
from Astar import AStarPlanner

# import SLAM components
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf1 import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco
from slam.mapping_utils import MappingUtils

sys.path.insert(0,"{}/network/".format(os.getcwd()))
sys.path.insert(0,"{}/network/scripts".format(os.getcwd()))
from network.scripts.detector import Detector
import TargetPoseEst as TPE
# import utility functions
from util.pibot import Alphabot
import util.measure as measure


def read_slam_targets(slam,targets):

    with open(targets, 'r') as f: 

        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = []

        try:
            gt_dict = json.load(f)                   
        except ValueError as e:
            with open(targets, 'r') as f:
                gt_dict = ast.literal_eval(f.readline())   

        for fruits in gt_dict:
            fruit_list.append(fruits)

        for fruit in fruit_list:
            fruit_true_pos.append([gt_dict[fruit]['x'], gt_dict[fruit]['y']])

        arucomap = MappingUtils()
        arucomap.load(slam)

        markers = arucomap.markers
        taglist = arucomap.taglist

        for i in range(0,len(taglist)):
            aruco_true_pos.append([markers[0][i], markers[1][i]])

        return fruit_list, fruit_true_pos, aruco_true_pos, taglist


def read_search_list(fruits_list):
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('search_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    all_fruits = ['redapple', 'greenapple', 'orange', 'mango', 'capsicum']

    missing_fruits = []
    for fruit in all_fruits: # list out all unknown fruits positions
        if (fruit in search_list) == False:
            missing_fruits.append(fruit)

    return search_list, missing_fruits


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
    delta_rot = np.arctan2(delta_y, delta_x) - robot_pose[0][2]
    angle = (delta_rot + np.pi) % (2 * np.pi) -np.pi
    turn_time1 = (abs(angle)*baseline) / (turn_vel*scale*2)
    
    if angle > 0:
        turn_direc1 = [-turn_vel, turn_vel]
    elif angle < 0:
        turn_direc1 = [turn_vel, -turn_vel]
    else:
        turn_direc1 = [0, 0]

    # get drive time (forward)
    drive_time = delta_dist / (scale * wheel_vel[1])
    
    # turn towards the waypoint
    if turn_time1 > 0.01:
        print("Turning for {} seconds speed {}".format(turn_time1, turn_direc1))
        move_1 = ppi.set_velocity([0, 1], tick=turn_direc1, time=turn_time1+0.05)
    img_1 = ppi.get_image()

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

    lv, rv, moving_time = move_1
    drive_meas = measure.Drive(lv, rv, moving_time)
    lms, aruco_img = aruco_det.detect_marker_positions(img_1)
    ekf.predict(drive_meas)
    ekf.update(lms)

    lv, rv, moving_time = move_2
    drive_meas = measure.Drive(lv, rv, moving_time)
    lms, aruco_img = aruco_det.detect_marker_positions(img_2)
    ekf.predict(drive_meas)
    ekf.update(lms)

    robot_pose = ekf.robot.state

    return robot_pose.T

def calib_Rob_Pose(baseline,scale,waypoint):
    # to define turn duration for calibration
    print("calibration robot pose")
    turn_vel = 15
    turn_time = (abs(30*np.pi/180)*baseline) / (turn_vel*scale*2)
    img_1 = ppi.get_image()
    markers_seen = 0
    reached = 0
    pose_error = 0.5
    loops = 0

    # keep spinning to check position until robot detects at least 2 markers
    lms, aruco_img = aruco_det.detect_marker_positions(img_1)
    if len(lms) > 0:
        markers_seen += len(lms)
    while markers_seen < 3 or loops < 3:
        move_1 = ppi.set_velocity([0,1],tick=[turn_vel,-turn_vel],time=turn_time)
        img_1 = ppi.get_image()
        lv, rv, moving_time = move_1
        drive_meas = measure.Drive(lv, rv, moving_time)
        lms, aruco_img = aruco_det.detect_marker_positions(img_1)
        ekf.predict(drive_meas)
        ekf.update(lms)
        time.sleep(0.5)
        robot_pose = ekf.robot.state.T
        loops += 1/6
        if len(lms) > 0:
            markers_seen += len(lms)
    #if waypoint[0][0]-pose_error<robot_pose[0][0]<waypoint[0][0]+pose_error and waypoint[0][1]-pose_error<robot_pose[0][1]<waypoint[0][1]+pose_error:
        reached = 1
        
    return robot_pose, reached

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

def det_fruits(img, camera_matrix, target_fruit, robot_pose, scale, baseline):
    detector_output, network_vis = detector.yolo_detection(img)

    # get estimated positions of detected fruits
    completed_img_dict = TPE.get_image_info(detector_output, ekf.robot.state)
    target_pose_dict = TPE.estimate_pose(camera_matrix, completed_img_dict)
    for fruit in target_pose_dict:
        if fruit == target_fruit:
            move_1, move_2, img_1, img_2 = drive_to_point(scale, baseline, waypoint,robot_pose)
            robot_pose = get_robot_pose(move_1, move_2, img_1, img_2)

def plot_graph(ox, oy, taglist, fruits_true_pos, fruits_list, robot_pose, rx, ry, space):
    ax = plt.gca()
    ax.scatter(ox, oy, marker='s', color='C0', s=40)
    for i in range(len(taglist)):
        ax.text(ox[i]+0.05, oy[i]+0.05, taglist[i], color='C0', size=8)
    for i in range(0,len(fruits_list)):
        ax.scatter(fruits_true_pos[i][0], fruits_true_pos[i][1], marker='o', color='C1', s=40)
        ax.text(fruits_true_pos[i][0]+0.05, fruits_true_pos[i][1]+0.05, fruits_list[i], color='C1', size=8)

    plt.plot(robot_pose[0][0],robot_pose[0][1], 'b8')

    plt.xlabel("X"); plt.ylabel("Y")
    plt.xticks(space); plt.yticks(space)
    plt.grid()
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show(block=False)
    plt.pause(2)
    plt.close()
# main loop
if __name__ == "__main__":
    parser = argparse.ArgumentParser("Fruit searching")
    parser.add_argument("--map", type=str, default='lab_output/transformed_map.txt')
    parser.add_argument("--targets", type=str, default='lab_output/targets_transformed.txt')
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=8000)
    args, _ = parser.parse_known_args()

    ppi = Alphabot(args.ip,args.port)
    #detector = Detector(args.ckpt, use_gpu=False)
    ppi.set_Buzzer(0)

    #initialize variables
    x, y = 0.0, 0.0
    sx, sy = 0.0, 0.0
    waypoint = [[0.0, 0.0, 0.0]]
    robot_pose = [[sx, sy, 0.0]]
    move_1 = np.empty((0,3))
    move_2 = np.empty((0,3))
    img_1 = np.zeros([480, 640, 3], dtype=np.uint8)
    img_2 = np.zeros([480, 640, 3], dtype=np.uint8)
    grid_size, robot_radius, offset = 0.15, 0.12, 0.0  # [m]
    marker_size = 0.04
    space = np.array([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])

    # obstacle coordinates
    ox, oy = [], []
    ox_new, oy_new = [], []
    fruits_true_pos_seq = [[],[],[]] # number of fruits to drive to and sequence
    missing_fruits_pos = [[],[]]

    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos, taglist = read_slam_targets(args.map, args.targets)
    search_list, missing_fruits = read_search_list(fruits_list)

    missing_fruits_pos, fruits_true_pos_seq, ox_new, oy_new = AStarPlanner.setup_obstacles(search_list, fruits_list, fruits_true_pos_seq, fruits_true_pos, missing_fruits, missing_fruits_pos, aruco_true_pos, ox, oy, ox_new, oy_new, marker_size)
    
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

    # check initial postion of robot(in case robot wasn't placed perfectly at 0,0,0)
    img_1 = ppi.get_image()
    robot_pose = get_robot_pose([0,0,0],[0,0,0],img_1,img_1)
    #robot_pose = calib_Rob_Pose(baseline, scale)
    sx, sy = robot_pose[0][0], robot_pose[0][1]
    print(sx,sy)
    while len(fruits_true_pos_seq) != 0: # while still have fruit to drive to
        num_moves = 0
        ppi.set_velocity([0, 0], tick=[0,0])
        time.sleep(1)
        
        # goal position (position of fruits)
        gx = fruits_true_pos_seq[0][0]
        gy = fruits_true_pos_seq[0][1]

        sx,sy,rx,ry = path_planning(sx,sy,gx,gy,ox_new,oy_new,grid_size,robot_radius,offset)

        plot_graph(ox, oy, taglist, fruits_true_pos, fruits_list, robot_pose, rx, ry, space)

        for j in range(len(rx)-2,-1,-1): # move robot to specified waypoint
            waypoint[0][0] = rx[j]
            waypoint[0][1] = ry[j]

            if [waypoint[0][0],waypoint[0][1]] == [rx[0],ry[0]] or j == 1:
                move_1, move_2, img_1, img_2 = drive_to_point(scale, baseline, waypoint,robot_pose)
                robot_pose = get_robot_pose(move_1, move_2, img_1, img_2)
                print(j, " robot pose: ", robot_pose, " waypoint: ",waypoint)

                #robot_pose, reached = calib_Rob_Pose(baseline, scale, waypoint) # check robot position
                reached = 1
                if reached == 0:
                    print("Trying again: ", robot_pose)
                elif reached == 1:
                    print("Fruit Reached")
                    ppi.set_Buzzer(1)
                    ppi.set_velocity([0, 0], tick=[0,0])
                    fruits_true_pos_seq.pop(0)
                    time.sleep(1)
                    ppi.set_Buzzer(0)
                    sx, sy = robot_pose[0][0], robot_pose[0][1]
                break

            if [waypoint[0][0],waypoint[0][1]] != [rx[0],ry[0]]: # if haven't reached destination
                move_1, move_2, img_1, img_2 = drive_to_point(scale, baseline, waypoint,robot_pose)
                robot_pose = get_robot_pose(move_1, move_2, img_1, img_2)
                print(j, " robot pose: ", robot_pose, " waypoint: ",waypoint)
                ppi.set_velocity([0, 0], tick=[0,0])

            time.sleep(1)

            num_moves+=1
            #if num_moves >3:
                #robot_pose = calib_Rob_Pose(baseline, scale)
                #plot_graph(ox, oy, taglist, fruits_true_pos, fruits_list, robot_pose, rx, ry, space)
                #break
            
            plot_graph(ox, oy, taglist, fruits_true_pos, fruits_list, robot_pose, rx, ry, space)

            ppi.set_velocity([0, 0], tick=[0,0])
            
    ppi.set_velocity([0, 0], tick=[0,0])
        