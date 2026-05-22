# Host Setup
**It is strongly recommended that you run _setup_host.sh_ on your host machine before installing Century.**

## UDEV rules
All udev rule files under `etc/udev/rules.d` in the current folder are examples to show your how to map the specific devices to standard device names that Century recognizes below:
* 99-usbtty.rules: it maps 3 USB devices to novatel devices that Century uses to receive GPS signals.
* 99-webcam.rules (**deprecated in Century 3.5 and later versions**): it maps 3 USB camera devices with different names to what Century uses for different perception tasks: obstacle, traffic light and lane mark detections. All 3 cameras are front facing cameras, and wide angle cameras are normally used for obstacle and lane mark detection. Long focal lens camera is used for traffic light detection.
* 99-asucam.rules (**available in Century 3.5 and later versions**): it maps all FPD-Link cameras attached to ASU to what Century uses for different perception tasks at different positions. Please modify this file after you check your camera attributes and connect the cameras whose attributes correspond to the correct devices to be mapped.

## Identifying camera attributes
Please use the following command to check the device (_e.g. video0_) attributes:

`udevadm info --attribute-walk --name /dev/video0`

