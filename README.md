# Hardware Tests

This runs hardware tests.

## How to run test

```bash
git clone https://github.com/DroneResponse/hardware-tests.git
cd hardware-tests
docker-compose up
```

## Old Readme

The rest of these docs are somewhat outdated now.

**TODO:** update everything after this line.

## How to run tests


First, start `mavros`:

```bash
roslaunch dr_hardware_tests mavros.launch
```

Next run the hardware tests

```bash
rosrun dr_hardware_tests indoor_sensors.py
rosrun dr_hardware_tests sensors.py
rosrun dr_hardware_tests arm.py
rosrun dr_hardware_tests hover.py

```

## Install PX4

Please download the latest stable release of PX4 (works with our project) using the following command:

```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
```

## How to run tests connecting to a serial device

We are using docker to run the tests on a Jetson. Docker has been installed on the Jetson to be used for the first drone using this [documentation](https://docs.docker.com/engine/install/ubuntu/). A docker image named **test** containing all the necessary packages and the code has also been built on that Jetson. Using that image, the code can be run using the following instructions:

In a terminal, we need to execute the following command to run the docker image and enter a docker container:

```bash
sudo docker run --rm -it --device=/dev/ttyUSB0 test

```
The terminal will enter the docker container's work directory which is catkin_ws. The terminal should show something similar to **root@c5b2f3541681:/catkin_ws#**. We need to copy the hostname. Then in a separate terminal, the follwing command needs to be executed:

```bash
sudo docker exec -it c5b2f3541681 bash

```
Then the second terminal will also enter the same docker container. **/dev/ttyUSB0 test** is used for serial USB connection. For other types of connection, this is a good [reference](https://mavlink.io/en/mavgen_python/). USB connection can be verified by this command:

```bash
ls /dev/ttyUSB*

```
To run the code, in the first temrinal, launch the mavros and ROS master using this command:

```bash
roslaunch dr_hardware_tests mavros.launch

```
The launch file is located under catkin_ws/src/dr_hardware_tests/launch/mavros.launch. The code has been tested using baudrate of 921600 following the instructions on this [documentation](https://docs.px4.io/master/en/companion_computer/pixhawk_companion.html). For any other baud rate, the following line in the launch file needs to be modified:

```bash
<arg name="fcu_url" default="/dev/ttyUSB0:921600"/>

```
In the second terminal, the tests can be run using these commands:

```bash
rosrun dr_hardware_tests indoor_sensors.py
rosrun dr_hardware_tests sensors.py
rosrun dr_hardware_tests arm.py
rosrun dr_hardware_tests hover.py
rosrun dr_hardware_tests box.py

```
If there are changes to the code in the repository, the following instructions can be followed to create a new docker image.

The Dockerfile is copying a work directory named catkin_ws which is on the Jetson's home directory. In catkin/src/dr_hardware_tests, the code needs to be updated. The easiest way to do it is to delete the old **dr_hardware_tests** directory, copy the **hardware-tests** directory and rename it to **dr_hardware_tests**. Then the followng commands can be used to build the new image.

```bash
sudo docker build .

```
After successculd build, the image will have an id similar to c5b2f3541681. For ease of use, we can tag it to something (e.g. test) using this command:

```bash
sudo docker tag c5b2f3541681 test

```
If the change needs to be made is minor, an easier way is to change the code inside the docker container. This will require the change to be made every time the docker image is run. The change can be made using vim editor which is installed in the container. So the launch file can be edited by navigating to catkin_ws/src/dr_hardware_tests/launch/ and then running:

```bash
sudo vim mavros.launch

```

## Install MQTT

Install the mosquitto MQTT broker on the Jetpack on Jetson (Not on docker).

```bash
sudo apt-get install mosquitto
sudo apt-get install mosquitto-clients

```












