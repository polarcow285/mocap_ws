import sys
if sys.prefix == '/home/nl455/miniconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nl455/mocap_ws/install/mocap4r2cli'
