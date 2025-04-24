import numpy as np

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

eul = np.array([0.0,0.0,np.pi/2])
quat = Euler2Quaternion(eul)
eul2 = Quaternion2Euler(quat)
print(f"eul {eul}, quat {quat}, eul2 {eul2}")