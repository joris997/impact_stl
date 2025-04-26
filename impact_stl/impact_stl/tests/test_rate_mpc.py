import rclpy
import time
from rclpy.node import Node

from impact_stl.impact_stl.ff_rate_mpc_impact import SpacecraftRateMPC
import numpy as np
from impact_stl.models.spacecraft_rate_model import SpacecraftRateModel

class TestRateMPC(Node):
    def __init__(self):
        super().__init__('test_impact_mpc')
        print("starting rate-mpc test")
        # create mpc object
        model = SpacecraftRateModel()
        mpc = SpacecraftRateMPC(model)
        
        x0 = np.array([0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0,
                       1.0, 0.0, 0.0, 0.0]).reshape((10,1))
        setpoints = []
        for i in range(mpc.N+1):
            setpoints.append(x0 + i*0.1)
        x_pred, u_pred = mpc.solve(x0,setpoints,
                                   verbose=True)

        x_pred2, u_pred2 = mpc.solve(x0,setpoints,
                                     initial_guess={'X': x_pred, 'U': u_pred},
                                     verbose=True)
        print(f"x_pred: {x_pred}")
        print(f"u_pred: {u_pred}")


def main(args=None):
    rclpy.init(args=args)

    impact_mpc = TestRateMPC()
    rclpy.spin(impact_mpc)
    impact_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import sys
# import numpy as np
# import matplotlib.pyplot as plt
# from rate_mpc import SpacecraftRateMPC

# sys.path.append('..')
# from models.spacecraft_rate_model import SpacecraftRateModel

# model = SpacecraftRateModel()
# mpc = SpacecraftRateMPC(model)

# x0 = np.array([1.0, 0.0, 0.0,           # position
#                0.0, 0.0, 0.0,           # velocity
#                1.0, 0.0, 0.0, 0.0]).reshape((10,1))     # quaternion
# setpoints = [x0+i*0.1 for i in range(mpc.N+1)]
# x_pred, u_pred = mpc.solve(x0,setpoints)
# time = np.linspace(0,mpc.Tf,mpc.N+1)

# # plot x_pred x, y, z in one column and dx, dy, dz in another column
# plt.subplots(2,3, figsize=(10, 10))

# for i in range(3):
#     plt.subplot(2,3, i+1)
#     plt.plot(time,x_pred[i, :])
#     plt.plot(time,x_pred[i, :],'bo')
#     plt.title(f'x[{i}]')

# for i in range(3):
#     plt.subplot(2,3, i+4)
#     plt.plot(time,x_pred[i+3, :])
#     plt.plot(time,x_pred[i+3, :],'bo')
#     plt.title(f'dx[{i}]')

# plt.show()