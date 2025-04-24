#!/usr/bin/env python
import numpy as np

from rclpy.clock import Clock
from planner.utilities.beziers import value_bezier, eval_t
from geometry_msgs.msg import PoseStamped


def Quaternion2Euler(q):
    # quaternion in shape [w,x,y,z]
    # euler angles in [pitch,roll,yaw]
    return np.array([
        np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2)),
        np.arcsin(2*(q[0]*q[2] - q[3]*q[1])),
        np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))
    ])

def Euler2Quaternion(euler):
    # quaternion in shape [w,x,y,z]
    # euler angles in [pitch,roll,yaw]
    #! UNTESTED
    (pitch, roll, yaw) = euler
    return np.array([
        np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2),
        np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2),
        np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2),
        np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
    ])

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = Clock().now().to_msg()
    pose_msg.header.frame_id = frame_id
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = float(position[0])
    pose_msg.pose.position.y = float(position[1])
    pose_msg.pose.position.z = float(position[2])
    return pose_msg

def BezierCurve2NumpyArray(bezier_curve):
    # BezierCurve consists of an array of points. point.x, point.y, and point.z are the 
    # control point. the array gives all the control points. Pack this in a numpy array
    print(f"bezier_curve {bezier_curve}")
    ncp = len(bezier_curve.x_cp)
    # hvar is a Bezier curve but only has values in x_cp, so we need to check if the z_cp is empty
    if len(bezier_curve.y_cp) != ncp:
        control_points = np.zeros((1,ncp))
        for i in range(ncp):
            control_points[0,i] = bezier_curve.x_cp[i]
        return control_points
    else:
        control_points = np.zeros((3,ncp))
        for i in range(ncp):
            control_points[0,i] = bezier_curve.x_cp[i]
            control_points[1,i] = bezier_curve.y_cp[i]
            control_points[2,i] = bezier_curve.z_cp[i] # orientation
        return control_points

def BezierPlan2NumpyArray(bezier_plan):
    # create a tmp function to get the control points of a bezier, for all bezier segments
    def convert_curves(curves):
        return [BezierCurve2NumpyArray(curve) for curve in curves]
    
    return {
        'rvar': convert_curves(bezier_plan.rvar),
        'drvar': convert_curves(bezier_plan.drvar),
        'hvar': convert_curves(bezier_plan.hvar),
        'dhvar': convert_curves(bezier_plan.dhvar)
    }

def VerboseBezierPlan2NumpyArray(bezier_plan):
    plan = BezierPlan2NumpyArray(bezier_plan)
    plan['ids'] = bezier_plan.ids
    plan['other_names'] = bezier_plan.other_names
    return plan

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