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
To run the SITL or hardware simulations, I have provided a DockerFile in `DockerFiles`. This image requires approximately 55Gb of storage as it installs, among others, `ros2`, `gazebo` from source, and `px4-space-systems`.

```docker build -t impact_stl:latest -f DockerFiles/Dockerfile .```

```docker run -it --gpus all --network=host --ipc=host -e DISPLAY=$DISPLAY -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /dev/dri:/dev/dri -e XDG_RUNTIME_DIR=/tmp/runtime-root --runtime=nvidia -v /home/none/gits/PROJECTS/impact_stl:/home/px4space/space_ws/src/impact_stl --name impact_stl_cont impact_stl```