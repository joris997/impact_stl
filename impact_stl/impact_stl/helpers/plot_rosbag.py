#!/usr/bin/env python3

# thanks to David Dörner @doernerd
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

import numpy as np
import matplotlib.pyplot as plt

# For custom messages, we have to register the message type. Otherwise
# the reader doesn't know what to do.
def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

# Get all default ROS2 message types
typestore = get_typestore(Stores.ROS2_HUMBLE)
add_types = {}

# Directory with the custom message definitions. Assumes the ros WS is in the
# home directory
# message_dir = '/home/px4space/PX4/ros2_ws/src/px4_msgs/msg/'
message_dir = '/home/px4space/PX4/ros2_ws/src/px4_msgs/msg/'

# All custom message files have to be specified here
custom_msgs = ['VehicleAngularVelocity.msg',
               'VehicleAttitude.msg',
               'VehicleLocalPosition.msg',
               'VehicleOdometry.msg']
for custom_msg in custom_msgs:
    pathstr = message_dir + custom_msg
    msgpath = Path(pathstr)
    msgdef = msgpath.read_text(encoding='utf-8')
    add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))
typestore.register(add_types)

# print(f"add_types: {add_types}")


# Path to the directory with the rosbag. Not the rosbag itself
# due to the way ros2 records bags now.
# file ='/home/px4space/space_ws/rosbag_1'
file = '/home/px4space/space_ws/rosbag2_2025_01_15-18_52_46'

# robot_names = ['snap', 'crackle']
robot_names = ['snap', 'crackle', 'pop']
# robot_names = ['crackle']
robot_states = {robot_name:{} for robot_name in robot_names}

for robot_name in robot_names:
    # from VehicleOdometry
    robot_states[robot_name]['t_odom'] = []
    robot_states[robot_name]['q1'] = []
    robot_states[robot_name]['q2'] = []
    robot_states[robot_name]['q3'] = []
    robot_states[robot_name]['q4'] = []
    robot_states[robot_name]['dq1'] = []
    robot_states[robot_name]['dq2'] = []
    robot_states[robot_name]['dq3'] = []

    # from VehicleLocalPosition
    robot_states[robot_name]['t_pos'] = []
    robot_states[robot_name]['x'] = []
    robot_states[robot_name]['y'] = []
    robot_states[robot_name]['z'] = []
    robot_states[robot_name]['dx'] = []
    robot_states[robot_name]['dy'] = []
    robot_states[robot_name]['dz'] = []
    robot_states[robot_name]['ddx'] = []
    robot_states[robot_name]['ddy'] = []
    robot_states[robot_name]['ddz'] = []

    # from VehicleLocalPositionGroundTruth
    robot_states[robot_name]['t_pos_gz'] = []
    robot_states[robot_name]['x_gz'] = []
    robot_states[robot_name]['y_gz'] = []
    robot_states[robot_name]['z_gz'] = []
    robot_states[robot_name]['dx_gz'] = []
    robot_states[robot_name]['dy_gz'] = []
    robot_states[robot_name]['dz_gz'] = []
    robot_states[robot_name]['ddx_gz'] = []
    robot_states[robot_name]['ddy_gz'] = []
    robot_states[robot_name]['ddz_gz'] = []

    # from reference_path and predicted_path
    robot_states[robot_name]['t_path'] = []
    robot_states[robot_name]['ref_path'] = []
    robot_states[robot_name]['pred_path'] = []


t0 = None

# Create reader instance and open for reading.
with Reader(file) as reader:
    # # Topic and msgtype information is available on .connections list.
    # for reader_connection in reader.connections:
    #     print(reader_connection.topic, reader_connection.msgtype)

    # Iterate over messages.
    for robot_name in robot_names:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == f'/{robot_name}/fmu/out/vehicle_odometry':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                robot_states[robot_name]['t_odom'].append(msg.timestamp/1e6)
                robot_states[robot_name]['dq1'].append(msg.angular_velocity[0])
                robot_states[robot_name]['dq2'].append(msg.angular_velocity[1])
                robot_states[robot_name]['dq3'].append(msg.angular_velocity[2])

                if t0 is None:
                    t0 = msg.timestamp/1e6

            if connection.topic == f'/{robot_name}/fmu/out/vehicle_local_position':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                robot_states[robot_name]['t_pos'].append(msg.timestamp/1e6)
                robot_states[robot_name]['x'].append(msg.x)
                robot_states[robot_name]['y'].append(msg.y)
                robot_states[robot_name]['z'].append(msg.z)
                robot_states[robot_name]['dx'].append(msg.vx)
                robot_states[robot_name]['dy'].append(msg.vy)
                robot_states[robot_name]['dz'].append(msg.vz)
                robot_states[robot_name]['ddx'].append(msg.ax)
                robot_states[robot_name]['ddy'].append(msg.ay)
                robot_states[robot_name]['ddz'].append(msg.az)

            if connection.topic == f'/{robot_name}/fmu/out/vehicle_local_position_gz':
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                robot_states[robot_name]['t_pos_gz'].append(msg.timestamp/1e6)
                robot_states[robot_name]['x_gz'].append(msg.x)
                robot_states[robot_name]['y_gz'].append(msg.y)
                robot_states[robot_name]['z_gz'].append(msg.z)
                robot_states[robot_name]['dx_gz'].append(msg.vx)
                robot_states[robot_name]['dy_gz'].append(msg.vy)
                robot_states[robot_name]['dz_gz'].append(msg.vz)
                robot_states[robot_name]['ddx_gz'].append(msg.ax)
                robot_states[robot_name]['ddy_gz'].append(msg.ay)
                robot_states[robot_name]['ddz_gz'].append(msg.az)

            if f'{robot_name}/impact_stl/predicted_path' in connection.topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                robot_states[robot_name]['t_path'].append(msg.header.stamp.sec)
                robot_states[robot_name]['pred_path'].append(np.array([msg.poses[0].pose.position.x,
                                                                    msg.poses[0].pose.position.y,
                                                                    msg.poses[0].pose.position.z]))
            if f'{robot_name}/impact_stl/reference_path' in connection.topic:
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                robot_states[robot_name]['ref_path'].append(np.array([msg.poses[0].pose.position.x,
                                                                    msg.poses[0].pose.position.y,
                                                                    msg.poses[0].pose.position.z]))

    print("Bag read")

# convert time to start at zero when the simulation starts
for robot_name in robot_names:
    robot_states[robot_name]['t_pos'] = [t-t0 for t in robot_states[robot_name]['t_pos']]
    robot_states[robot_name]['t_pos_gz'] = [t-t0 for t in robot_states[robot_name]['t_pos_gz']]
    robot_states[robot_name]['t_odom'] = [t-t0 for t in robot_states[robot_name]['t_odom']]
    robot_states[robot_name]['t_path'] = [t-t0 for t in robot_states[robot_name]['t_path']]
# convert everything to numpy arrays
for robot_name in robot_names:
    for key, value in robot_states[robot_name].items():
        robot_states[robot_name][key] = np.array(robot_states[robot_name][key])

# then cleanup the data by removing all indices for which t<0, these are misalignments
#TODO: determine if this is really the way to do things, seems to incur jumps in time...
for robot_name in robot_names:
    ts = robot_states[robot_name]['t_pos']
    pos_idxs = np.where(ts>=0)[0]
    # print(pos_idxs)
    for key, value in robot_states[robot_name].items():
        if key not in ['t_pos','x','y','z','dx','dy','dz','ddx','ddy','ddz']:
            continue
        try:
            robot_states[robot_name][key] = robot_states[robot_name][key][pos_idxs]
        except:
            # might pass when never filled in!
            pass

for robot_name in robot_names:
    ts = robot_states[robot_name]['t_pos_gz']
    pos_idxs = np.where(ts>=0)[0]
    # print(pos_idxs)
    for key, value in robot_states[robot_name].items():
        if key not in ['t_pos_gz','x_gz','y_gz','z_gz','dx_gz','dy_gz','dz_gz','ddx_gz','ddy_gz','ddz_gz']:
            continue
        try:
            robot_states[robot_name][key] = robot_states[robot_name][key][pos_idxs]
        except:
            # might pass when never filled in!
            pass

for robot_name in robot_names:
    ts = robot_states[robot_name]['t_odom']
    pos_idxs = np.where(ts>=0)[0]
    # print(pos_idxs)
    for key, value in robot_states[robot_name].items():
        if key not in ['t_odom','q1','q2','q3','q4','dq1','dq2','dq3']:
            continue
        try:
            robot_states[robot_name][key] = robot_states[robot_name][key][pos_idxs]
        except:
            # might pass when never filled in!
            pass


fig, axs = plt.subplots(3,3)
fig.suptitle("FreeFlyer State")

for robot_name in ['snap','crackle','pop']:
    xref = [ref[0] for ref in robot_states[robot_name]['ref_path']]
    xpred = [pred[0] for pred in robot_states[robot_name]['pred_path']]
    yref = [-ref[1] for ref in robot_states[robot_name]['ref_path']]
    ypred = [-pred[1] for pred in robot_states[robot_name]['pred_path']]

    axs[0,0].plot(robot_states[robot_name]['y'],robot_states[robot_name]['x'])
    axs[0,0].plot(yref,xref)
    axs[0,0].set_xlabel('y pos [m]')
    axs[0,0].set_ylabel('x pos [m]')
    axs[0,0].set_aspect('equal')
    axs[0,0].grid(True)

    # pos
    axs[0,1].plot(robot_states[robot_name]['t_pos'],robot_states[robot_name]['x'], label=robot_name)
    # axs[0,1].plot(robot_states[robot_name]['t_path'],xref, label='ref', linestyle='--')
    # axs[0,1].plot(robot_states[robot_name]['t_path'],xpred, label='pred', linestyle='--')
    axs[0,1].set_xlabel('time [s]')
    axs[0,1].set_ylabel('x-pos [m]')
    axs[0,1].grid(True)
    axs[0,1].legend()

    axs[0,2].plot(robot_states[robot_name]['t_pos'],robot_states[robot_name]['y'], label=robot_name)
    # axs[0,2].plot(robot_states[robot_name]['t_path'],yref, label='ref', linestyle='--')
    # axs[0,2].plot(robot_states[robot_name]['t_path'],ypred, label='pred', linestyle='--')
    axs[0,2].set_xlabel('time [s]')
    axs[0,2].set_ylabel('y-pos [m]')
    axs[0,2].grid(True)
    axs[0,2].legend()


    # vel
    axs[1,1].plot(robot_states[robot_name]['t_pos'],robot_states[robot_name]['dx'], label=robot_name)
    axs[1,1].plot(robot_states[robot_name]['t_pos_gz'],robot_states[robot_name]['dx_gz'], label=robot_name+' gz')
    axs[1,1].set_xlabel('time [s]')
    axs[1,1].set_ylabel('x-vel [m/s]')
    axs[1,1].grid(True)
    axs[0,1].legend()

    axs[1,2].plot(robot_states[robot_name]['t_pos'],robot_states[robot_name]['dy'], label=robot_name)
    axs[1,2].plot(robot_states[robot_name]['t_pos_gz'],robot_states[robot_name]['dy_gz'], label=robot_name+' gz')
    axs[1,2].set_xlabel('time [s]')
    axs[1,2].set_ylabel('y-vel [m/s]')
    axs[1,2].grid(True)
    axs[0,1].legend()


    # acc
    axs[2,1].plot(robot_states[robot_name]['t_pos'],robot_states[robot_name]['ddx'], label=robot_name)
    # axs[2,1].plot(robot_states[robot_name]['t_pos_gz'],robot_states[robot_name]['ddx_gz'], label=robot_name+' gz')
    axs[2,1].set_xlabel('time [s]')
    axs[2,1].set_ylabel('x-acc [m/s^2]')
    axs[2,1].grid(True)
    axs[0,1].legend()

    axs[2,2].plot(robot_states[robot_name]['t_pos'],robot_states[robot_name]['ddy'], label='snap')
    # axs[2,2].plot(robot_states[robot_name]['t_pos_gz'],robot_states[robot_name]['ddy_gz'], label='snap gz')
    axs[2,2].set_xlabel('time [s]')
    axs[2,2].set_ylabel('y-acc [m/s^2]')
    axs[2,2].grid(True)
    axs[0,1].legend()

plt.show()




