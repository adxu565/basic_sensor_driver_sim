#!/bin/bash

# function to make sure running processes are cleaned up
cleanup() {
    echo "Cleaning up all relevant background processes"
    if ps -p $SOCAT_PID > /dev/null; then
        #echo "Killing socat with pid: $SOCAT_PID"
        kill -9 $SOCAT_PID
    else
        :
        #echo "pid: $SOCAT_PID is already not running!"
    fi

    if ps -p $IMU_DRIVER_PID > /dev/null; then
        #echo "Killing imu_driver with pid: $IMU_DRIVER_PID"
        kill -9 $IMU_DRIVER_PID
    else
        :
        #echo "pid: $IMU_DRIVER_PID is already not running!"
    fi   

    if ps -p $IMU_SIM_PID > /dev/null; then
        #echo "Killing imu_sim with pid: $IMU_SIM_PID"
        kill -9 $IMU_SIM_PID
    else
        :
        #echo "pid: $IMU_SIM_PID is already not running!"
    fi   

    if ps -p $BASHPID > /dev/null; then
        #echo "Killing run_drivers_and_sensors.sh with pid: $BASHPID"
        kill -9 $BASHPID
    else
        :
        #echo "pid: $BASHPID is already not running!"
    fi 
}

# ensure cleanup
trap cleanup EXIT
trap cleanup INT
trap cleanup TERM

# start socat in the background
socat -d -d PTY,raw,echo=0 PTY,raw,echo=0 > socat_output.log 2>&1 &
SOCAT_PID=$!

# allow time to connect
sleep 5

# extract port names from output
ports=$(grep -oP '/dev/pts/\d+' socat_output.log)

# arbitrarily assign first port to the driver, 2nd to the sensor.
# start the driver first, and then start the sensor (both running in the background)
ports=$(echo "$ports" | tr '\n' ' ')
read -r port1 port2 <<< "$ports"
echo "Port 1: $port1"
echo "Port 2: $port2"

./build/imu_driver "$port1" &
IMU_DRIVER_PID=$!

# use xterm to open a new window to visualize the imu
xterm -hold -e "./build/imu_sim "$port2"" &
IMU_SIM_PID=$!

# now sleep and let the programs run in the background
# ctrl-c to exit if imu_driver or imu_sim are running indefinitely.
# use make debug for a temporary simulation. 

while true; do
    if ! ps -p $IMU_DRIVER_PID > /dev/null || ! ps -p $IMU_SIM_PID > /dev/null; then
        cleanup
        exit
    fi
    sleep 1
done