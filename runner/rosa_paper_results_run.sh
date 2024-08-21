#!/bin/bash

MTYPE="time"
GUI="false"
RUNS=100

MANAGER="rosa"
./rosa_runner.sh $GUI $MANAGER $MTYPE $RUNS

MANAGER="metacontrol"
./rosa_runner.sh $GUI $MANAGER $MTYPE $RUNS

MANAGER="bt"
./rosa_runner.sh $GUI $MANAGER $MTYPE $RUNS

MANAGER="none"
./rosa_runner.sh $GUI $MANAGER $MTYPE $RUNS

MANAGER="rosa"
MTYPE="extended"
./rosa_runner.sh $GUI $MANAGER $MTYPE $RUNS

MANAGER="bt"
./rosa_runner.sh $GUI $MANAGER $MTYPE $RUNS
