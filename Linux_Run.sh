#!/bin/bash
sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
python3 ./Code/main.py