import json
import SLAM_eval as ev
import numpy as np
# read turemap made in generateGroundTruth and convert to slam.txt format, necessary to load aruco markers into slam for robot pose estimation

# get marker positions in ground truth
groundtruth ="M4_true_map.txt"
gt_aruco = ev.parse_groundtruth(groundtruth)

# Seperate tag number and marker coordinate
pos = []
taglist = []
for i in gt_aruco:
    taglist.append(i)
    pos.append(gt_aruco[i].T)

# seerate x and y coordinates into their own lists
x = []
y = []
for i in range(0,len(pos)):
    x.append(pos[i][0][0])
    y.append(pos[i][0][1])

# combine markers and create empty covariance
markers = [x,y]
covariance = np.zeros((20,20))

# export data into file
map_attributes = {"tag":taglist,
                    "markers":markers,
                    "covariance":covariance.tolist()}
with open("true_map.txt",'w') as map_file:
    json.dump(map_attributes, map_file, indent=2)
