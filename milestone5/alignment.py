from __future__ import print_function
import ast
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from slam.mapping_utils import MappingUtils

def parse_user_map(fname : str) -> dict:
    with open(fname, 'r') as f:
        try:
            usr_dict = json.load(f)                   
        except ValueError as e:
            with open(fname, 'r') as f:
                usr_dict = ast.literal_eval(f.readline()) 
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            aruco_dict[tag] = np.reshape([usr_dict["map"][0][i],usr_dict["map"][1][i]], (2,1))
    return aruco_dict

def get_fruit_pos():
    with open('lab_output/targets.txt', 'r') as file:
        data = json.load(file)

    # get x and y values for each marker
    x_values = {}
    y_values = {}
    fruit_list = []

    for key, value in data.items():
        x_values[key] = value["x"]
        y_values[key] = value["y"]
        fruit_list.append(key)

    x_values = np.array(list(x_values.items()))
    y_values = np.array(list(y_values.items()))
    x_coordinates = []
    y_coordinates = []

    for subarray in x_values:
        x_coordinates.append(subarray[1])

    for subarray in y_values:
        y_coordinates.append(subarray[1])

    fruit_pos = []

    for i in range(len(x_values)):
        fruit_pos.append([x_coordinates[i], y_coordinates[i]])

    x_coordinates = np.array([float(string) for string in x_coordinates])
    y_coordinates = np.array([float(string) for string in y_coordinates])

    return x_coordinates, y_coordinates, fruit_list

def get_aruco(aruco0 : dict):
    points0 = []
    keys = []
    for key in range(1,11):
        points0.append(aruco0[key])
        keys.append(key)
    return keys, np.hstack(points0)

def apply_transform(theta, t_x, t_y, x, y):
    # Convert theta to radians
    theta_rad = theta* np.pi/180

    x_transformed = x * np.cos(theta_rad) - y * np.sin(theta_rad)
    y_transformed = x * np.sin(theta_rad) + y * np.cos(theta_rad)
    x_transformed += t_x
    y_transformed += t_y
    return x_transformed, y_transformed

us_aruco = parse_user_map('lab_output/slam_map.txt')
taglist,[aruco_x,aruco_y] = get_aruco(us_aruco)
robot_x =  1.2638
robot_y = -0.89007
space = [-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6]
fruit_x, fruit_y, fruit_list = get_fruit_pos()

# create the initial scatter plot
fig, ax = plt.subplots()
scatter = ax.scatter(aruco_x, aruco_y, marker='x', color='C1', s=100)
text = []
for i in range(len(taglist)): 
    text.append(ax.text(aruco_x[i]+0.05, aruco_y[i]+0.05, taglist[i], color='C0', size=12))
    
scatter2 = ax.scatter(robot_x, robot_y, marker='o', color='C2', s=100)
scatter3 = ax.scatter(fruit_x, fruit_y, marker='o', color='C3', s=20)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Alignment Tool')
plt.title('Alignment Tool')
plt.xlabel('X')
plt.ylabel('Y')
ax.set_xticks(space)
ax.set_yticks(space)
plt.legend(['Real','Pred'])
plt.grid()
# add sliders for rotation and translation
axcolor = 'lightgoldenrodyellow'
theta = 100
t_x = 0.4
t_y = 0.4

# rotation slider
ax_slider_theta = plt.axes([0.25, 0.02, 0.65, 0.03], facecolor=axcolor)
slider_theta = Slider(ax_slider_theta, 'Theta', 0, 360, valinit=0)

# translation sliders
ax_slider_t_x = plt.axes([0.25, 0.06, 0.65, 0.03], facecolor=axcolor)
slider_t_x = Slider(ax_slider_t_x, 'X', -3, 3, valinit=0)

ax_slider_t_y = plt.axes([0.25, 0.10, 0.65, 0.03], facecolor=axcolor)
slider_t_y = Slider(ax_slider_t_y, 'Y', -3, 3, valinit=0)

# update scatter plot based on slider values
def update(val):
    theta = slider_theta.val
    t_x = slider_t_x.val
    t_y = slider_t_y.val
    x_transformed, y_transformed = apply_transform(theta, t_x, t_y, aruco_x, aruco_y)
    robot_x_transformed, robot_y_transformed = apply_transform(theta, t_x, t_y, robot_x, robot_y)
    fruit_x_transformed, fruit_y_transformed = apply_transform(theta, t_x, t_y, fruit_x, fruit_y)
    scatter.set_offsets(np.column_stack((x_transformed, y_transformed)))
    scatter2.set_offsets(np.column_stack((robot_x_transformed, robot_y_transformed)))
    scatter3.set_offsets(np.column_stack((fruit_x_transformed, fruit_y_transformed)))
  
    for i in range(len(taglist)):
        text[i].set_x(x_transformed[i]+0.05)
        text[i].set_y(y_transformed[i]+0.05)
    fig.canvas.draw_idle()

# attach the update function to slider events
slider_theta.on_changed(update)
slider_t_x.on_changed(update)
slider_t_y.on_changed(update)

plt.show()

theta = slider_theta.val
t_x = slider_t_x.val
t_y = slider_t_y.val
print(t_x, t_y, theta)
x_transformed, y_transformed = apply_transform(theta, t_x, t_y, aruco_x, aruco_y)
fruitx_transformed, fruity_transformed = apply_transform(theta, t_x, t_y, fruit_x, fruit_y)

aruco_transformed = [x_transformed.tolist(), y_transformed.tolist()]

# save calibrated map
trans_map = MappingUtils()
trans_map.load('lab_output/slam.txt')
trans_map.markers = aruco_transformed
trans_map.taglist = taglist
trans_map.covariance = np.zeros((20,20))
trans_map.save('lab_output/transformed_map.txt')

# save calibrated targets
target_pose_dict = {}
for i in range(0,len(fruit_list)):
    target_pose_dict[fruit_list[i]] = {'x':fruitx_transformed[i], 'y':fruity_transformed[i]}
with open('lab_output/targets_transformed.txt', 'w') as fo:
    json.dump(target_pose_dict, fo)