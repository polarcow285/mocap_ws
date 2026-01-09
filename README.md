# Setup for users with no admin permissions on conda
**Create a new conda environment**

```bash
conda create -n ros_env -c conda-forge -c robostack-kilted ros-kilted-desktop
```

Activate the environment

```bash
conda activate ros_env
```

Pip install python dependencies

```bash
pip3 install setuptools
```

**Workspace setup**

```bash
mkdir -p ~/mocap_ws/src
cd ~/mocap_ws/src
```

**Clone repositories**

```bash
# Clone OptiTrack driver (includes dependency repos)
cd ~/mocap_ws/src
git clone https://github.com/polarcow285/mocap_ws.git

# Import dependencies using vcs
vcs import < mocap4ros2_optitrack/dependency_repos.repos
# (This installed a duplicate "msgs" folder for me): vcs import < mocap4r2/dependency_repos.repos  # If mocap4r2 exists
```

**Install and setup rosdep (have to do it this way because we have no permissions in sudo)**

```bash
pip install rosdep
```

```bash
# Create a local directory structure
mkdir -p ~/ros_etc/ros/rosdep/sources.list.d

# Point rosdep to this new location
export ROS_ETC_DIR=~/ros_etc/ros
export ROSDEP_SOURCE_PATH=~/ros_etc/ros/rosdep/sources.list.d
```

Then run `rosdep init` and `rosdep update`

**Install System Dependencies**

```bash
conda install -c robostack-kilted -c conda-forge ros-kilted-ament-cmake-clang-format
rosdep install --from-paths src --ignore-src -r -y --os=ubuntu:24.04
```

**Building the workspace**

To install `colcon` and its common extensions using `pip`, you should use the following command: 

```bash
python3 -m pip install -U colcon-common-extensions
```

Install **LTTng Userspace Tracer** libraries

```bash
conda install -c conda-forge lttng-ust
export LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH
```

Build the workspace

```bash
cd ~/mocap_ws
colcon build --symlink-install
```

**Source the workspace**
```source ~/mocap_ws/install/setup.bash```

# Stream mocap data via ROS and visualize
Activate the ROS environment and source installation packages.

Source the workspace:

```source ~/mocap_ws/install/setup.bash```

Repeat for two terminal windows:
- `cd ~/mocap_ws`
- `conda activate ros_env` 

Launch driver node (connects to Motive server): make sure Motive multicast is enabled (white stream icon)

- In terminal 1: `ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py`
- In terminal 2: `ros2 lifecycle set /mocap4r2_optitrack_driver_node activate`

To launch visualization of markers and rigid bodies, in terminal 2:

```python3 ~/mocap_ws/plot_markers.py```
You may need to `conda install scipy` when running visualization script.

# Collect human demonstration data
1. Launch driver nodes to connect to Motive server as described above.
2. In the OpenTeach repository, run `python teleop.py`.
3. Open the VR app 'KinovaBot' in the MetaQuest 2. Using your left hand, pinch your middle finger and thumb together. The border should be blue, indicating Arm + Hand mode.
4. Ensure you are wearing two mocap markers around your wrist.
5. Run `python tracker.py` to begin collecting human demonstration data (data collection begins immediately). Data is saved as a .npz 
6. Run `python visualize_tracking_data.py` to visualize data.
