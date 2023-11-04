import numpy as np
from slam.mapping_utils import MappingUtils
import ast
import matplotlib.pyplot as plt
import json

def parse_user_map(fname : str) -> dict:
    with open(fname, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            aruco_dict[tag] = np.reshape([usr_dict["map"][0][i],usr_dict["map"][1][i]], (2,1))
    
    points0 = []
    keys = []
    for key in aruco_dict:
        points0.append(aruco_dict[key])
        keys.append(key)

    return keys,np.hstack(points0)


def get_targets(targets):
    with open(targets, 'r') as f: 

        fruit_list = []
        fruit_true_pos = []

        try:
            gt_dict = json.load(f)                   
        except ValueError as e:
            with open(targets, 'r') as f:
                gt_dict = ast.literal_eval(f.readline())   

        for fruits in gt_dict:
            fruit_list.append(fruits)

        for fruit in fruit_list:
            fruit_true_pos.append([gt_dict[fruit]['x'], gt_dict[fruit]['y']])

    return fruit_list, fruit_true_pos

def remove_unknown(taglist, aruco_vec):
    idx = []
    x = aruco_vec[0]
    y = aruco_vec[1]
    for tag in taglist:
        if tag > 10:
            idx.append(taglist.index(tag))

    for i in range(len(idx)-1,-1,-1):
        taglist.remove(taglist[idx[i]])
        x = np.delete(x, idx[i])
        y = np.delete(y, idx[i])

    aruco_vec = np.array([x,y])
    return taglist, aruco_vec

def transform_map(theta, x, points):
    """
        :param theta: float in radian
        :param x:[[x transform][y transform]]
        :param points: [[x][y]]
    """
    
    c, s = np.cos(-theta), np.sin(-theta)
    R = np.array(((c, -s), (s, c)))
    points210 = [points[0][1:],points[1][1:]]

    points_transformed1 =  R @ points210 - x
    #points_transformed =  R @ points - x
    points_transformedy = points_transformed1[1]
    points_transformedy = np.append(points[1][0], points_transformedy)
    points_transformedx = points_transformed1[0]
    points_transformedx = np.append(points[0][0], points_transformedx)
    points_transformed = [points_transformedx.tolist(), points_transformedy.tolist()]
    return points_transformed

if __name__ == '__main__':
    # get origin coordinates
    robot_pose_origin = np.empty((3,1))
    origin = np.empty((2,1))
    
    robot_pose_origin[0] = 0.0 # +left -right
    robot_pose_origin[1] = 0.0 # +down -up
    robot_pose_origin[2] = 0.0 # +cw -acw

    # load slam map and targets
    taglist, aruco_vec = parse_user_map('lab_output/slam.txt')
    taglist, aruco_vec = remove_unknown(taglist, aruco_vec)
    

    # calibrate slam map
    theta = float(robot_pose_origin[2])
    origin[0] = float(robot_pose_origin[0])
    origin[1] = float(robot_pose_origin[1])
    # calibrate fruit positions
    aruco_transformed = transform_map(theta, origin, aruco_vec) 

    ax = plt.gca()
    space = np.array([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
    ax.scatter(0, 0, marker='x', color='C1', s=40)
    ax.scatter(aruco_transformed[0], aruco_transformed[1], marker='o', color='C0', s=40)
    for i in range(len(taglist)):
        ax.text(aruco_transformed[0][i]+0.05, aruco_transformed[1][i]+0.05, taglist[i], color='C0', size=8)
    plt.xlabel("X"); plt.ylabel("Y")
    plt.xticks(space); plt.yticks(space)
    plt.grid()
    plt.pause(0.001)
    plt.show()

    # save calibrated map
    trans_map = MappingUtils()
    trans_map.load('lab_output/slam_map.txt')
    trans_map.markers = aruco_transformed
    trans_map.taglist = taglist
    trans_map.covariance = np.zeros((20,20))
    trans_map.save('lab_output/slam_map.txt')
