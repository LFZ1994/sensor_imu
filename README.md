# sensor_imu
WeChangeTech USB/Uart IMU modul ROS package
Step1: clone this package to you workspace/src fold
Step2: catkin_make your workspace
Step3: roscd sensor_imu/script/
Step4: sudo ./udev.sh
Step5: connect IMU device to your PC via USB cable
Step6: roslaunch sensor_imu sensor_imu.launch
Now you can get /imu and /mag topic
Note:Step 1~4 need do only once when you install this package

