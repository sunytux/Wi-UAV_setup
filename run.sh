#!/bin/bash

BUILD_DIR=~/Onboard-SDK/build/
BIN_DIR=~/Onboard-SDK/build/bin/

if [ "$1" == "compile" ]; then
	echo Compiling...
	cd $BUILD_DIR
	make -j djiosdk-base-station

elif [ "$1" == "run" ]; then
	echo Running...
	cd $BIN_DIR
	sudo ./djiosdk-base-station UserConfig.txt "$2"

else 
	echo Unknown argument.
	echo Use compile or run
fi
