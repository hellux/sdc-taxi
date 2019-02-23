# Autonomous RC-taxi

<img src="https://github.com/hellux/sdc-taxi/raw/master/.taxi.jpg" width="500">

A project where a RC-car autonomously drives through a map to a destination on
command.

## Description
A client is running on a PC. The remote client and the car are connected to a
WLAN through which the client sends commands to the car. A mission command
consists of a list of commands such as "park, stop, ignore, enter (roundabout),
exit" which will be executed upon each stop line on the map. To travel from a
point A to B the client calculates a list of commands to reach B via the
closest path and sends the list to the car.

### Car
The car uses a camera to detect and follow the road and detect stop lines. A
distance sensor at the front detects obstacles.

The car consists of three modules that communicate via an SPI bus. The
communication module is a Raspberry Pi that hosts a TCP/IP server and listens
for commands from the client. The communication module also handles image
processing and the mission. The module receives sensor data from the sensor
module and sends control commands to the control module. Both the sensor
and control modules are ATMega 1284p AVRs. The control module implements a PID
controller for both steering and speed.
