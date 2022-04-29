# HOW TO BUILD:
#   docker build -f IntelAero.dockerfile .
# take note of the image hash

# HOW TO RUN ON INTEL AERO
# 1. turn off mavlink router:
#       systemctl stop mavlink-router
# 2. Run docker with "--privileged" flag:
#       docker run -it --rm --privileged 295685ab8dee bash
# 3. source the ros setup file:
#       source /opt/ros/noetic/setup.bash
# 4. start mavros with fcu_url:=/dev/ttyS1:921600 for example:
#       roslaunch dr_hardware_tests mavros.launch fcu_url:=/dev/ttyS1:921600 &
# 5. run a test:
#       rosrun dr_hardware_tests indoor_sensors.py

FROM ros:noetic-ros-base-focal 

RUN apt-get update && \
    apt-get install -y geographiclib-tools && \
    geographiclib-get-geoids egm96-5 

RUN apt-get update \
    && apt-get install --yes \
        python-is-python3 \
        python3 \
        python3-pip \
        python3-venv \
        python3-wheel \
        python3-dev \   
        python3-catkin-tools \
        ros-noetic-mavros \
        ros-noetic-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

COPY ./requirements.txt /requirements.txt
RUN pip install -r /requirements.txt


COPY . /catkin_ws/src/dr_hardware_tests
WORKDIR /catkin_ws
RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic
