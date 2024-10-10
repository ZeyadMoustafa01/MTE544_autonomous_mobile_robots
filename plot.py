import matplotlib.pyplot as plt
from utilities import FileReader
from typing import Dict, Any, List
import os
import numpy as np
import math

# obstacle_list = [
#         (5, 5, 1),
#         (3, 6, 2),
#         (3, 8, 2),
#         (3, 10, 2),
#         (7, 5, 2),
#         (9, 5, 2),
#         (8, 10, 1),
#         (6, 12, 1),
#     ]

obstacle_list = [
        (2, 2, 1),
        (6, 2, 2),
        (4, 4, 2),
        (8, 3, 1),
        (10, 4, 1),
        (5, 7, 2),
        (10, 10, 2),
        (6, 12, 1),
        (12, 7, 2)
        ]

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))


    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values])
    axes[0].set_title("state space")
    axes[0].grid()

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i])

    axes[1].legend()
    axes[1].grid()

    plt.show()
    
    
def plot_trajectory_information(information: Dict[str, Any], path: str=None):

    _, values=FileReader(information["file"]).read_file()


    time_unprocessed = [row[-1] for row in values]
    
    o = information["O"]
    g = information["G"]
    
    robot_path = np.load(file=f"data/obstacle{o}_goal{g}.npy")

    time = [time-time_unprocessed[0] for time in time_unprocessed]
    imu_ax = [val[0] for val in values]
    imu_ay = [val[1] for val in values]
    kf_ax = [val[2] for val in values]
    kf_ay = [val[3] for val in values]
    kf_vx = [val[4] for val in values]
    kf_w = [val[5] for val in values]
    x = [val[6] for val in values]
    y = [val[7] for val in values]
    
    x_path = [val[0] for val in robot_path]
    y_path = [val[1] for val in robot_path]
    
    deg = list(range(0, 360, 5))
    deg.append(0)
    
    plt.figure(figsize = (8, 6))

    plt.plot(x, y, 'b', label="Path executed by robot")
    plt.plot(x_path, y_path, "--r", label="Path from RRT*")
    for (ox, oy, size) in obstacle_list:
        xl = [ox + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [oy + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, "k")

    plt.xlabel(xlabel="X position [m]")
    plt.ylabel(ylabel="Y position [m]")
    plt.title(label=f"X vs Y for Trajectory for EKF Obstacle {o}, GoalPose {g}")
    plt.grid(visible=True)
    plt.legend()
    plt.savefig(f"{path}/x_vs_y_o{o}_g{g}.png")




import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)
    data_points = [
        {"file": "robotPose_O2_G2.csv", "O": "2", "G": "2"},
    ]
    
    for row in data_points:
        path = "Graphs"
        if not os.path.isdir(s=path):
            os.mkdir(path=path)
        plot_trajectory_information(information=row, path=path)


