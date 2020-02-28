#!/bin/bash

echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="sensor_imu"' >/etc/udev/rules.d/sensor_imu_module.rules

service udev reload
sleep 1
service udev restart
