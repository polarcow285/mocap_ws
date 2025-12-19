Activate the ROS environment and source installation packages.
Repeat for two terminal windows:
- `cd ~/mocap_ws`
- `conda activate ros_env` 

Launch driver node (connects to Motive server): make sure Motive multicast is enabled (white stream icon)

- In terminal 1: `ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py`
- In terminal 2: `ros2 lifecycle set /mocap4r2_optitrack_driver_node activate`

To launch visualization step, in terminal 2:

```python3 ~/mocap_ws/plot_marks.py```
