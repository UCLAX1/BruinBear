#!/bin/bash

echo "bringing CAN interface down..."
sudo ip link set can1 down

echo "bringing CAN interface up..."
sudo ip link set can1 up type can bitrate 1000000

echo "sending message on CAN interface..."
cansend can1 123#DEADBEEF

echo "CAN interface status:"
ip link show can1