Using imu_tools: http://wiki.ros.org/imu_tools?distro=melodic \
Madgwick filter: http://wiki.ros.org/imu_filter_madgwick \
\
commands:\
rosrun rosserial_python serial_node.py /dev/ttyACM0\
rosrun imu_filter_madgwick imu_filter_node
