#!/bin/bash

echo "bringing CAN interface down..."
sudo ip link set can0 down

echo "bringing CAN interface up..."
sudo ip link set can0 up type can bitrate 1000000

echo "sending message on CAN interface..."
cansend can0 123#DEADBEEF

echo "CAN interface status:"
ip link show can0