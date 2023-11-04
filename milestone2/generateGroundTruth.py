
############### run this scrpt using the command: ###################################
#-------------- python generateGroundTruth.py <output file name> -------------------#
############### without the angle brackets ##########################################

import numpy as np
import matplotlib.pyplot as plt
import math
import json

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("outputfilename", type=str, help='output filename')
    args = parser.parse_args()
   
    space = np.array([-1.6, -1.2, -0.8, -0.4, 0, 0.4, 0.8, 1.2, 1.6])    
    
    fig = plt.figure()
    plt.plot(0,0,'rx')
    plt.xlabel("X"); plt.ylabel("Y")
    plt.xticks(space); plt.yticks(space)
    plt.grid()
    
    # Variables, p will contains clicked points, idx contains current point that is being selected
    px, py = [], []
    idx = 0
    fruit = ['redapple', 'greenapple', 'orange', 'mango', 'capsicum']
    fruit_color = [[128, 0, 0], [155, 255, 70], [255, 85, 0], [255, 180, 0], [0, 128, 0]]
    
    for i in range(len(fruit_color)):
        for j in range(3):
            fruit_color[i][j] /= 255
    
    def round_nearest(x, a):
        return round(round(x / a) * a, -int(math.floor(math.log10(a))))
    
    # pick points
    def onclick(event):
        global idx
        
        x = round_nearest(event.xdata, 0.4)
        y = round_nearest(event.ydata, 0.4)
        
        if event.button == 1:
            # left mouse click       
            px.append(x)
            py.append(y)
            idx += 1       
        elif event.button == 3:
            # right click, delete point
            del px[-1]
            del py[-1]
            idx -= 1
        
        plt.clf()
        plt.plot(0,0,'rx')
        # plt.scatter(px[:idx],py[:idx], color='C0')
        for i in range(len(px)):
            if i < 10:
                plt.scatter(px[i],py[i], color='C0')
                plt.text(px[i]+0.05, py[i]+0.05, i+1, color='C0', size=12)
            else:
                plt.scatter(px[i],py[i], color=fruit_color[i-10])
                plt.text(px[i]+0.05, py[i]+0.05, i+1, color=fruit_color[i-10], size=12)
        plt.xlabel("X"); plt.ylabel("Y")
        plt.xticks(space); plt.yticks(space)
        plt.grid()
        plt.show()
    
    print("Specify points on the grid, close figure when done.") 
    ka = fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    
    d = {}
    for i in range(len(px)):
        if i < 10:
            d["aruco" + str(i+1) + "_0"] = {"x": px[i], "y":py[i]}
        else:
            d[fruit[i-10] + "_0"] = {"x": px[i], "y":py[i]}
        
    with open(args.outputfilename+'.txt', 'w') as f:
        json.dump(d, f, indent=4)