# impact_stl
Code for the paper "Collaborative Object Transportation in Space via Impact Interactions"

# Before you start
The code, while being a ROS2 package in its entirety, consists of two distinct parts;
- `impact_stl/planner` which generates the desired motion plans and writes them to `.csv` files
- `impact_stl/impact_stl` which is the ROS2 package that reads the `.csv` files and executes them in SITL or on the real platform.

The only proprietary dependency is `gurobi` which is only required for `impact_stl/planner` to solve the Mixed-Integer Problem. If you do not want or cannot obtain a `gurobi` license, there are generated `.csv` files in `impact_stl/impact_stl/planner/plans` for all the scenarios described in the paper, and more!


# Installation
## Just the planner
If you have a `gurobi` license and you are interested in only generating the motion plans, you can build a virtual environment using the `environment.yml` file in `impact_stl/planner`

```conda env create -f environment.yml```


## The whole deal
To run the SITL or hardware simulations, I have provided a DockerFile in `DockerFiles`. This image requires approximately 15Gb of storage as it installs, among others, `ros2`, `gazebo`, and `px4-space-systems`.

```docker build -t impact_stl:latest -f DockerFiles/Dockerfile .```

```docker run -it --gpus all --network=host --ipc=host -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri -e XDG_RUNTIME_DIR=/tmp/runtime-root --runtime=nvidia -v /home/none/gits/PROJECTS/impact_stl:/home/px4space/space_ws/src/impact_stl --name impact_stl_cont impact_stl```

The last argument links your local clone of the `impact_stl` repository to the Docker container. You can change the path to your local clone of the repository.

# Running the code
## Planner
To run the planner, you need to activate the conda environment and run the `main.py` script. 
The script allows you to change robustness type and scenario. See the `World.py` and `Spec.py` for details on how to change the scenarios. 

In the `impact_stl` directory, run the following commands:

```conda activate impact_stl```

```python main.py```

## Simulation
The simulator requires several components to be running. We list them here:
- `QGroundControl` which is the ground control station for the PX4 autopilot, allowing us to arm, disarm, and change the control mode of the vehicle.
- `microros` which is the micro-ROS agent that allows us to communicate with the PX4 autopilot.
- `sitl launch file` which is the launch file that starts the PX4 autopilot in software-in-the-loop (SITL) mode.
- `scenario launch file` which is the launch file that starts the all the controllers, planners, impact detectors etc.
- `start launch file` which sends the global signal that the simulation should start. Also records the rosbag.

### QGroundControl
In the home directory, run the following command to start QGroundControl:
```./startQGC```

### Micro-ROS
In any directory, run the following command to start the micro-ROS agent:
```ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888```

### SITL launch file
Go to the `space_ws` directory, and build the workspace:
```colcon build```

Source the workspace:
```source install/setup.bash```

Then, run the following command to start the PX4 autopilot in SITL mode:
```ros2 launch impact_stl sitl_obstacle_avoidance.launch.py```

### Scenario launch file
In the `impact_stl` directory, run the following command to start the scenario:
```ros2 launch impact_stl obstacle_avoidance.launch.py```

### Start launch file
In the `impact_stl` directory, run the following command to start the simulation:
```ros2 launch impact_stl start_scenario.launch.py```