#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enx00e04c82a7f7 down
sudo ifconfig enx00e04c82a7f7 up 192.168.123.162 netmask 255.255.255.0

# sudo ifconfig enx00e04c82a7f7 multicast
# sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enx00e04c82a7f7

