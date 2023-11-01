"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import numpy as np
import json
import ast

import matplotlib.pyplot as plt
#import auto_fruit_search as afs
 
show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                #print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = -2.0
        self.min_y = -2.0
        self.max_x = 2.0
        self.max_y = 2.0
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    
    def setup_obstacles(search_list, fruits_list, fruits_true_pos_seq, fruits_true_pos, missing_fruits, missing_fruits_pos, aruco_true_pos, ox, oy, ox_new, oy_new, marker_size):
        for i in range(0,len(search_list)):
            for j in range(0,len(fruits_list)):
                if search_list[i] == fruits_list[j]:
                    fruits_true_pos_seq[i] = fruits_true_pos[j]

        for i in range(0,len(missing_fruits)):
            for j in range(0,len(fruits_list)):
                if missing_fruits[i] == fruits_list[j]:
                    missing_fruits_pos[i] = fruits_true_pos[j]

        for marker in range(0,len(aruco_true_pos)):
            ox.append(aruco_true_pos[marker][0])
            oy.append(aruco_true_pos[marker][1])
        for fruit in range(0,len(missing_fruits_pos)):
            ox.append(missing_fruits_pos[fruit][0])
            oy.append(missing_fruits_pos[fruit][1])

        for (x, y) in zip (ox, oy): # give dimension to obstacles in obstacle list
            ox_new.extend([x, x-marker_size, x-marker_size, x-marker_size, x+marker_size, x+marker_size, x+marker_size, x, x])
            oy_new.extend([y, y-marker_size, y, y+marker_size, y-marker_size, y, y+marker_size, y-marker_size, y+marker_size])

        return missing_fruits_pos, fruits_true_pos_seq, ox_new, oy_new
    
def main():
    print(__file__ + " start!!")
    space = np.array([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])
    rx_all = []
    ry_all = []
    fruits_true_pos_seq = [[],[],[]] # number of fruits to drive to and sequence
    missing_fruits_pos = [[],[]]


    fruits_list, fruits_true_pos, aruco_true_pos, taglist = afs.read_slam_targets('lab_output/transformed_map.txt','lab_output/targets_transformed.txt')
    search_list, missing_fruits = afs.read_search_list(fruits_list)


    print(missing_fruits_pos)
    grid_size = 0.15  # [m]
    robot_radius = 0.12  # [m]
    # distance to stop away from fruit
    offset = 0.0
    # obstacle coordinates
    ox = []
    oy = []
    ox_new = []
    oy_new = []
    # starting from origin in arena
    sx = 0.0
    sy = 0.0
    marker_size = 0.04

    missing_fruits_pos, fruits_true_pos_seq, ox_new, oy_new = AStarPlanner.setup_obstacles(search_list, fruits_list, fruits_true_pos_seq, fruits_true_pos, missing_fruits, missing_fruits_pos, aruco_true_pos, ox, oy, ox_new, oy_new, marker_size)
        
    if show_animation:  # pragma: no cover
        ax = plt.gca()
        ax.scatter(ox, oy, marker='o', color='C0', s=40)
        for i in range(len(taglist)):
            ax.text(ox[i]+0.05, oy[i]+0.05, taglist[i], color='C0', size=8)
        for i in range(0,len(search_list)):
            ax.scatter(fruits_true_pos_seq[i][0], fruits_true_pos_seq[i][1], color='C1', s=40)
            ax.text(fruits_true_pos_seq[i][0]+0.05, fruits_true_pos_seq[i][1]+0.05, search_list[i], color='C1', size=8)
        for i in range(0,len(missing_fruits)):
            ax.scatter(missing_fruits_pos[i][0], missing_fruits_pos[i][1], color='C3', s=40)
            ax.text(missing_fruits_pos[i][0]+0.05, missing_fruits_pos[i][1]+0.05, missing_fruits[i], color='C1', size=8)

        plt.xlabel("X"); plt.ylabel("Y")
        plt.xticks(space); plt.yticks(space)
        plt.grid()

    for i in range (0,len(search_list)): 
        # get goal position
        gx = fruits_true_pos_seq[i][0]
        gy = fruits_true_pos_seq[i][1]

        a_star = AStarPlanner(ox_new, oy_new, grid_size, robot_radius)
        # current output goes to position of fruit directly, need to adjust offset by radius around the fruit
        rx, ry = a_star.planning(sx, sy, gx, gy) # Returns coordinates from goal to start
        # get distance between rx,ry[0] and rx,ry[1]
        rx_dist = rx[0] - rx[1]
        ry_dist = ry[0] - ry[1]
        # find the intersecting point between the two where distance from robot to rx,ry = offset
        # update rx,ry[0] to intersect position
        rx[0] = rx[1] + rx_dist*offset/grid_size
        ry[0] = ry[1] + ry_dist*offset/grid_size
        # update start position
        sx = rx[1]
        sy = ry[1]

        rx_all.append(rx)
        ry_all.append(ry)

        print(rx)
        print(ry)

    plt.plot(rx_all[0], ry_all[0], "-r")
    plt.plot(rx_all[1], ry_all[1], "-g")
    plt.plot(rx_all[2], ry_all[2], "-b")
    plt.pause(0.001)

    plt.show()

if __name__ == '__main__':
    import auto_fruit_search as afs
    main()