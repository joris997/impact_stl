import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation

import sys

sys.path.append("/home/none/space_ws/src/impact_stl/impact_stl/planner")
from utilities.beziers import value_bezier, eval_t
from utilities.read_write_plan import csv_to_zonotopes, csv_to_plan
from utilities.beziers import get_derivative_control_points_gurobi


def interpolate_bezier(plan, t):
    idx, s = eval_t(plan['hvar'], t)
    return {
        'q': value_bezier(plan['rvar'][idx], s),
        'dq': value_bezier(plan['drvar'][idx], s)/value_bezier(plan['dhvar'][idx], s),
        'h': value_bezier(plan['hvar'][idx], s),
        'dh': value_bezier(plan['dhvar'][idx], s),
        'id': plan['ids'][idx],
        'other_name': plan['other_names'][idx]
    }

class CSV2Dict(object):
    """
    Helper class to convert PlotJuggler CSV data to dictionary for plotting.
    Useful when some data has missing timestamps.
    """
    def __init__(self, data, name="snap", tmax=None):
        self.data = data
        self.name = name
        self.tmax = tmax
        self.df = pd.read_csv(data)

    def get_local_position_data(self):
        data = {'t': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': [], 'ax': [], 'ay': [], 'az': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/fmu/out/vehicle_local_position/x",
                                f"/{self.name}/fmu/out/vehicle_local_position/y",
                                f"/{self.name}/fmu/out/vehicle_local_position/z",
                                f"/{self.name}/fmu/out/vehicle_local_position/vx",
                                f"/{self.name}/fmu/out/vehicle_local_position/vy",
                                f"/{self.name}/fmu/out/vehicle_local_position/vz",
                                f"/{self.name}/fmu/out/vehicle_local_position/ax",
                                f"/{self.name}/fmu/out/vehicle_local_position/ay",
                                f"/{self.name}/fmu/out/vehicle_local_position/az"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])
        data['x'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/x"])
        data['y'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/y"])
        data['z'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/z"])
        data['vx'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/vx"])
        data['vy'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/vy"])
        data['vz'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/vz"])
        data['ax'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/ax"])
        data['ay'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/ay"])
        data['az'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position/az"])
        return data
    
    def get_local_position_gz_data(self):
        data = {'t': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': [], 'ax': [], 'ay': [], 'az': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/fmu/out/vehicle_local_position_gz/x",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/y",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/z",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/vx",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/vy",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/vz",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/ax",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/ay",
                                f"/{self.name}/fmu/out/vehicle_local_position_gz/az"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])
        data['y'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/x"])
        data['x'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/y"])
        data['z'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/z"])
        data['vy'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/vx"])
        data['vx'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/vy"])
        data['vz'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/vz"])
        data['ay'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/ax"])
        data['ax'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/ay"])
        data['az'] = list(df_cleaned[f"/{self.name}/fmu/out/vehicle_local_position_gz/az"])
        return data
    
    def get_mpc_reference_data(self):
        data = {'t': [], 'x': [], 'y': [], 'z': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/position/x",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/position/y",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/position/z"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])
        data['x'] = list(df_cleaned[f"/{self.name}/impact_stl/reference_path/poses[0]/pose/position/x"])
        data['y'] = list(df_cleaned[f"/{self.name}/impact_stl/reference_path/poses[0]/pose/position/y"])
        data['z'] = list(df_cleaned[f"/{self.name}/impact_stl/reference_path/poses[0]/pose/position/z"])
        return data

    def get_mpc_position_data(self):
        data = {'t': [], 'x': [], 'y': [], 'z': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/position/x",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/position/y",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/position/z",]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])
        data['x'] = list(df_cleaned[f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/position/x"])
        data['y'] = list(df_cleaned[f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/position/y"])
        data['z'] = list(df_cleaned[f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/position/z"])
        return data

    def get_mpc_attitude_data(self):
        data = {'t': [], 'roll': [], 'pitch': [], 'yaw': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/x",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/y",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/z",
                              f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/w"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])

        # Convert quaternion to numpy array for Rotation class
        quats = np.array(df_cleaned[[f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/x",
                                    f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/y",
                                    f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/z",
                                    f"/{self.name}/impact_stl/predicted_path/poses[0]/pose/orientation/w"]])

        euler = Rotation.from_quat(quats).as_euler('xyz', degrees=True)
        data['roll'] = list(euler[:, 0])
        data['pitch'] = list(euler[:, 1])
        data['yaw'] = list(euler[:, 2])
        return data

    def get_mpc_attitude_reference_data(self):
        data = {'t': [], 'roll': [], 'pitch': [], 'yaw': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/x",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/y",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/z",
                              f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/w"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])
        # Convert quaternion to numpy array for Rotation class
        quats = np.array(df_cleaned[[f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/x",
                                     f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/y",
                                     f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/z",
                                     f"/{self.name}/impact_stl/reference_path/poses[0]/pose/orientation/w"]])

        euler = Rotation.from_quat(quats).as_euler('xyz', degrees=True)
        data['roll'] = list(euler[:, 0])
        data['pitch'] = list(euler[:, 1])
        data['yaw'] = list(euler[:, 2])
        return data

    def get_rate_setpoint_data(self):
        data = {'t': [], 'rate_x': [], 'rate_y': [], 'rate_z': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/fmu/in/vehicle_rates_setpoint/thrust_body[0]",
                              f"/{self.name}/fmu/in/vehicle_rates_setpoint/thrust_body[1]",
                              f"/{self.name}/fmu/in/vehicle_rates_setpoint/thrust_body[2]",
                              f"/{self.name}/fmu/in/vehicle_rates_setpoint/roll",
                              f"/{self.name}/fmu/in/vehicle_rates_setpoint/pitch",
                              f"/{self.name}/fmu/in/vehicle_rates_setpoint/yaw"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"])
        data['thrust_x'] = list(df_cleaned[f"/{self.name}/fmu/in/vehicle_rates_setpoint/thrust_body[0]"])
        data['thrust_y'] = list(df_cleaned[f"/{self.name}/fmu/in/vehicle_rates_setpoint/thrust_body[1]"])
        data['thrust_z'] = list(df_cleaned[f"/{self.name}/fmu/in/vehicle_rates_setpoint/thrust_body[2]"])
        data['omega_x'] = list(df_cleaned[f"/{self.name}/fmu/in/vehicle_rates_setpoint/roll"])
        data['omega_y'] = list(df_cleaned[f"/{self.name}/fmu/in/vehicle_rates_setpoint/pitch"])
        data['omega_z'] = list(df_cleaned[f"/{self.name}/fmu/in/vehicle_rates_setpoint/yaw"])
        return data
    
    def get_odom_data(self,offset=0):
        data = {'t': [], 'rate_x': [], 'rate_y': [], 'rate_z': []}
        df_cleaned = self.df[["__time",
                              f"/{self.name}/odom/pose/pose/position/x",
                              f"/{self.name}/odom/pose/pose/position/y",
                              f"/{self.name}/odom/pose/pose/position/z"]]
        df_cleaned = df_cleaned.dropna()
        df_cleaned["__time"] = df_cleaned["__time"] - df_cleaned["__time"].iloc[0]
        data['t'] = list(df_cleaned["__time"] + offset)
        data['x'] = list(df_cleaned[f"/{self.name}/odom/pose/pose/position/x"])
        data['y'] = list(df_cleaned[f"/{self.name}/odom/pose/pose/position/y"])
        data['z'] = list(df_cleaned[f"/{self.name}/odom/pose/pose/position/z"])

        return data


def Plan2Dict(path, name="snap", tmax=None):
    plandict = {}

    rvar, hvar, ids, other_names = csv_to_plan(name, path=path)
    plandict = {"rvar":rvar, "hvar":hvar, "ids":ids, "other_names":other_names}
    plandict["drvar"] = [get_derivative_control_points_gurobi(rvar[i]) for i in range(len(rvar))]
    plandict["dhvar"] = [get_derivative_control_points_gurobi(hvar[i]) for i in range(len(hvar))]
    N = 200
    ts = np.linspace(plandict["hvar"][0][0,0], plandict["hvar"][-1][-1,0], N)
    plandict["heval"] = []
    plandict["qeval"] = []
    plandict["dqeval"] = []
    for t in ts:
        plani = interpolate_bezier(plandict,t)
        plandict["heval"].append(plani["h"])
        plandict["qeval"].append(plani["q"])
        plandict["dqeval"].append(plani["dq"])

    return plandict

def Zonotopes2Dict(path, name="snap", tmax=None):
    zonotopesdict = {}

    X0s, Xfs, ids, other_names = csv_to_zonotopes(name, path=path)
    zonotopesdict = {"X0s":X0s, "Xfs":Xfs, "ids":ids, "other_names":other_names}

    return zonotopesdict