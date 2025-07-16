import matplotlib.pyplot as plt
from matplotlib import rc
from matplotlib.axes import Axes
from matplotlib.patches import Patch
from matplotlib.gridspec import GridSpec
import sys
from contextlib import suppress

from pathlib import Path
import numpy as np
import scienceplots

from plot_helpers import CSV2Dict
from plot_helpers import Plan2Dict


# Set up plot style
plt.style.use(['science'])
rc('text', usetex=True)
rc('font', family='times', size=20)

plt.rcParams['legend.frameon'] = True             # Enable legend frame
plt.rcParams['legend.facecolor'] = 'white'        # Set background color
plt.rcParams['legend.edgecolor'] = 'white'        # Set border color
plt.rcParams['legend.framealpha'] = 1.0
plt.rcParams['legend.loc'] = 'best'


robots = ["snap","crackle","pop"]

# Read the .csv file from plotjuggler
# csv_path = str(Path.home()) + "/space_ws/rosbags/lab_test_1/rosbag2_plotjuggler.csv"
# csv_path = str(Path.home()) + "/space_ws/rosbags/obstacle_avoidance/best_case/rosbag2_plotjuggler.csv"
# csv_path = str(Path.home()) + "/space_ws/rosbags/throw_and_catch/best_case/rosbag2_plotjuggler.csv"
# csv_path = "/home/px4space/space_ws/rosbags/lab_test_2/rosbag2_plotjugger.csv"

# csv_path = "/home/px4space/space_ws/rosbags/pingpong/rosbag2_2025_01_30-18_47_57/rosbag2_plotjuggler.csv"
# csv_path = "/home/px4space/space_ws/rosbags/pingpong/rosbag2_2025_01_30-19_06_41/rosbag2_plotjuggler.csv"
# csv_path = "/home/px4space/space_ws/rosbags/pingpong/rosbag2_2025_01_30-19_23_11/rosbag2_plotjuggler.csv"

# csv_path = "/home/px4space/space_ws/rosbags/obstacle_avoidance/worst_case/rosbag2_plotjuggler.csv"

# PATHS FOR RSS BAGS
# csv_path = "/home/none/Downloads/rosbags/rss_bags/rosbag2_2025_05_19-19_57_32/rosbag2_plotjuggler.csv"
# csv_path = "/home/none/Downloads/rosbags/rss_bags/rosbag2_2025_05_19-20_07_59/rosbag2_plotjuggler.csv"
csv_path = "/home/none/Downloads/rosbags/rss_bags/rosbag2_2025_05_19-21_27_50/rosbag2_plotjuggler.csv"
csv2dict = {}
for robot in robots:
    csv2dict[robot] = CSV2Dict(csv_path, name=robot, tmax=60)

# # Read the .csv file from the planner 
# plan_path = str(Path.home()) + "/space_ws/rosbags/throw_and_catch/best_case/plans"
# csv2dict_plan = {}
# for robot in robots:
#     csv2dict_plan[robot] = Plan2Dict(plan_path, name=robot, tmax=60)

# # Read the .csv file for any zonotope files
# zonotope_path = str(Path.home()) + "/space_ws/rosbags/throw_and_catch/best_case/plans"
# csv2dict_zonotope = {}
# for robot in robots:
#     try:
#         csv2dict_zonotope[robot] = Plan2Dict(zonotope_path, name=robot, tmax=60)
#     except:
#         print(f"Zonotope file not found for {robot}")
    

# Read the replanned .csv files
replan_path = str(Path.home()) + "/space_ws/rosbags/throw_and_catch/best_case/replans"
csv2dict_replan = {}


# Start plotting
fig = plt.figure(figsize=(20, 10))
gs = GridSpec(2, 4, figure=fig)
# fig, axs = plt.subplots(2,4, figsize=(20, 10))
ax1 = fig.add_subplot(gs[0:2, 0:2])
ax2 = fig.add_subplot(gs[0, 2])
ax3 = fig.add_subplot(gs[1, 2])
ax4 = fig.add_subplot(gs[0, 3])
ax5 = fig.add_subplot(gs[1, 3])

robot_cl = ['k','k']
robot_ls = ['-','--']
object_cl = ['r']
object_ls = ['-']

for robot in robots:
    if robot == "snap":
        color = object_cl[0]
        ls = object_ls[0]
    elif robot == "crackle":
        color = robot_cl[0]
        ls = robot_ls[0]
    elif robot == "pop":
        color = robot_cl[1]
        ls = robot_ls[1]

    # Plot the reference path
    with suppress(KeyError): data_mpc_ref = csv2dict[robot].get_mpc_reference_data()
    data_local_pos = csv2dict[robot].get_local_position_data()

    # with suppress(KeyError): ax1.plot(data_mpc_ref['x'], data_mpc_ref['y'], color=color, ls='--')
    ax1.plot(data_local_pos['x'], [-y for y in data_local_pos['y']], color=color, ls=ls, label=fr"${robot}$")
    ax1.set_xlabel(r"x position [m]")
    ax1.set_ylabel(r"y position [m]")
    # ax1.set_xlim(0.25, 4.0)
    # ax1.set_ylim(-1.25, 1.75)
    # ax1.set_xticks(np.arange(0.5, 4, 0.5))
    # ax1.set_yticks(np.arange(-1.0, 2.0, 0.5))
    # ax1.set_xlim(0.3,3.7)
    # ax1.legend()
    ax1.grid(True)
    ax1.set_aspect('equal')

    # with suppress(KeyError): ax2.plot(data_mpc_ref['t'], data_mpc_ref['x'], color=color, ls='--')
    ax2.plot(data_local_pos['t'], [-y for y in data_local_pos['y']], color=color, ls=ls)
    ax2.set_xlabel(r"Time [s]")
    ax2.set_ylabel(r"y position [m]")
    # ax2.legend()
    ax2.grid(True)

    ax3.plot(data_local_pos['t'], [-y for y in data_local_pos['vy']], color=color, ls=ls)
    ax3.set_xlabel(r"Time [s]")
    ax3.set_ylabel(r"y velocity [m/s]")
    # ax3.legend()
    ax3.grid(True)

    # with suppress(KeyError): ax4.plot(data_mpc_ref['t'], data_mpc_ref['y'], color=color, ls='--')
    ax4.plot(data_local_pos['t'], data_local_pos['x'], color=color)
    ax4.set_xlabel(r"Time [s]")
    ax4.set_ylabel(r"x position [m]")
    # ax4.legend()
    ax4.grid(True)

    ax5.plot(data_local_pos['t'], data_local_pos['vx'], color=color)
    ax5.set_xlabel(r"Time [s]")
    ax5.set_ylabel(r"x velocity [m/s]")
    # ax5.legend()
    ax5.grid(True)

    fig.tight_layout()

# plt.savefig("/home/none/space_ws/src/impact_stl/impact_stl/results/plot.png")
plt.savefig("/home/none/space_ws/src/impact_stl/impact_stl/results/plot.svg")
# plt.show()