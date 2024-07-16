#!/bin/bash

source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14555 gcs_url:=udp-b://@