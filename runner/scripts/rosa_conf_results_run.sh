#!/bin/bash

MTYPE="time"
GUI="false"

MANAGER="rosa"
./rosa_runner.sh $GUI $MANAGER $MTYPE 20

MANAGER="metacontrol"
./rosa_runner.sh $GUI $MANAGER $MTYPE 20

MANAGER="none"
./rosa_runner.sh $GUI $MANAGER $MTYPE 20

MANAGER="random"
./rosa_runner.sh $GUI $MANAGER $MTYPE 20
