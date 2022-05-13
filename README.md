# Hardware Tests

This runs hardware tests.

## How to install

```bash
sudo apt install docker-compose
git clone https://github.com/DroneResponse/hardware-tests.git
```

## How to build

This could take a few minuets. You need an Internet connection.
```bash
cd hardware-tests
docker-compose build
docker pull ros:noetic-ros-core

```

## How to run tests

### Start mavros

Start mavros in the background. First, enter the `hardware-tests` directory:
```bash
cd hardware-tests
```
Then use one of the following approaches.

1. If using the Jetson and mavros connects to PX4 using `/dev/ttyUSB0` then run:
   ```bash
   docker-compose -f docker-compose.yaml -f docker-compose.jetson.yaml up -d mavros
   ```
2. If using the Simulator, then run:
   ```bash
   docker-compose -f docker-compose.yaml -f docker-compose.simulator.yaml up -d mavros
   ```
   This configures mavros to use `udp-b://:14540@14580` and it forwards the UDP packets to the container.

### Run the tests

Next run one or more tests:

```bash
docker-compose run --no-deps indoor_sensors
docker-compose run --no-deps sensors
docker-compose run --no-deps rc_failsafe
docker-compose run --no-deps gimbal
docker-compose run --no-deps arm
docker-compose run --no-deps hover
docker-compose run --no-deps box
docker-compose run --no-deps geofence
```

### Stop Mavros

When you're done, stop mavros:

```bash
docker-compose down
```

## Running on the Intel Aero

These instructions walk you through the special setup that's required to run the hardware tests on the Intel Aero.

Install [the latest version of Yacto Linux provided by Intel](https://github.com/intel-aero/meta-intel-aero/wiki/02-Initial-Setup#flashing).

Connect the drone to the internet. [Connect the drone to your WiFi network.](https://github.com/intel-aero/meta-intel-aero/wiki/08-Aero-Network-and-System-Administration#networking-wifi) If needed, fix DNS by editing `/etc/resolv.conf`:

```bash
vi /etc/resolv.conf
```

Add a line to the file, like so:

```txt
nameserver 192.168.1.1
```

### Install a newer version of Docker

Install `xz`. [Download the source tar ball](https://tukaani.org/xz/) to the drone. Extract it. Build it. Install it. For example:

```bash
wget --no-check-certificate  https://tukaani.org/xz/xz-5.2.5.tar.gz
tar xf xz-5.2.5.tar.gz 
cd xz-5.2.5
./configure
make
make install
xz --version
```

[Install the docker binaries.](https://docs.docker.com/engine/install/binaries/) Download the binaries from [download.docker.com/linux/static/stable/](https://download.docker.com/linux/static/stable/). For example:

```bash
wget https://download.docker.com/linux/static/stable/x86_64/docker-20.10.9.tgz
tar xf docker-20.10.9.tgz 
systemctl stop docker
cp docker/* /usr/bin/
systemctl start docker
```

### Install the `docker compose` binary plugin

Install the `docker compose` 2.X binary. [Instructions here](https://docs.docker.com/compose/install/#install-the-binary-manually).

```bash
mkdir -p /usr/local/lib/docker/cli-plugins
curl -SL https://github.com/docker/compose/releases/download/v2.5.0/docker-compose-linux-x86_64 -o /usr/local/lib/docker/cli-plugins/docker-compose
chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
docker compose version
```

### Starting the programs

Start mavros in the background with:

```bash
docker compose -f docker-compose.yaml -f docker-compose.intel-aero.yaml up -d mavros
```

Then to run the tests:

```bash
docker compose run --no-deps indoor_sensors
docker compose run --no-deps sensors
docker compose run --no-deps rc_failsafe
docker compose run --no-deps gimbal
docker compose run --no-deps arm
docker compose run --no-deps hover
docker compose run --no-deps box
docker compose run --no-deps geofence
```

To stop mavros:

```bash
docker compose down
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












