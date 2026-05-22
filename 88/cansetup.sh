#!/bin/bash
sudo busybox devmem 0x0c303000 32 0x0000C400
sudo busybox devmem 0x0c303008 32 0x0000C458
sudo busybox devmem 0x0c303010 32 0x0000C400
sudo busybox devmem 0x0c303018 32 0x0000C458
sleep 1

sudo modprobe can        #insert can module
sudo modprobe can-raw    #insert can protocol module
sudo modprobe mttcan     #insert nvidia can module
#lsmod to check the can status

# #1. config the can0 baudrate and loopback is active
# sudo ip link set can0 type can bitrate 500000

# #2. open the controller
# sudo ip link set up can0

# #3. show the controller status
# #ip -details link show can0

# #4. close the controller and set value of tx_queue_len
# sudo sh -c "sudo ifconfig can0 down"
# sudo sh -c "sudo ip link set can0 down"
# sudo sh -c "echo 1024 > /sys/class/net/can0/tx_queue_len"
# sleep 1
# sudo ip link set up can0

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000 
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 4000000 fd on
sudo ip link set can1 up
