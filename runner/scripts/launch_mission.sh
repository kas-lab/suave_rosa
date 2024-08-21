#!/bin/bash
source ~/.bashrc
source ~/suave_ws/install/setup.bash

if [ $# -lt 1 ]; then
	echo "usage: $0 adaptation_manager mission_type"
	echo example:
	echo "  "$0 "[metacontrol | random | none | rosa] [time | distance | extended]"
	exit 1
fi

MANAGER=""
MTYPE=""

if [ "$1" == "metacontrol" ] || [ "$1" == "random" ] || [ "$1" == "none" ] || [ "$1" == "bt" ] || [ "$1" == "rosa" ];
then
    MANAGER=$1
else
    echo "adaptation_manager invalid or missing"
    exit 1
fi

if [ "$2" == "time" ];
then
    MTYPE="time_constrained_mission"
elif [ "$2" == "distance" ];
then
    MTYPE="const_dist_mission"
elif [ "$2" == "extended" ];
then
    MTYPE="extended"
else
    echo "mission_type invalid or missing"
    exit 1
fi

FILE=$3
MCFILE=${FILE}"_mc_reasoning_time"
if [ "$1" == "metacontrol" ] || [ "$1" == "random" ] || [ "$1" == "none" ];
then
    ros2 launch suave_missions mission.launch.py adaptation_manager:=$MANAGER mission_type:=$MTYPE result_filename:=$FILE mc_reasoning_time_filename:=$MCFILE
elif [ "$1" == "bt" ];
then
  if [ "$MTYPE" == "extended" ];
  then
    ros2 launch suave_bt suave_bt_extended.launch.py result_filename:=$3
  else
    ros2 launch suave_bt suave_bt.launch.py result_filename:=$3
  fi
elif [ "$1" == "rosa" ]
then
    xfce4-terminal --execute typedb server &
    if [ "$MTYPE" == "extended" ];
    then
      ros2 launch suave_rosa_bt suave_rosa_extended_bt.launch.py mission_type:=$MTYPE result_filename:=$3
    else
      ros2 launch suave_rosa_bt suave_rosa_bt.launch.py mission_type:=$MTYPE result_filename:=$3
    fi
else
    echo "adaptation_manager invalid or missing"
    exit 1
fi
