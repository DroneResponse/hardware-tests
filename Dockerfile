#
# GEOID DOWNLOADER
FROM ubuntu:20.04 as geoid-downloader

RUN apt-get update && \
    apt-get install -y geographiclib-tools && \
    geographiclib-get-geoids egm96-5 

#
# BUILDER BASE
FROM ros:noetic-ros-base-focal AS base

RUN apt-key adv \
        --keyserver 'hkp://keyserver.ubuntu.com:80' \
        --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update \
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
# On the Jetson (and other ARM CPUs), Ruckig must be built from source.
# In order to compile, pybind11 and pybind11-global must be installed before running
# pip install ruckig (note: ruckig is installed by the requirements.txt file)
RUN pip install "pybind11==2.9.2" "pybind11-global==2.9.2"
RUN pip install -r /requirements.txt

#
# BUILDER
FROM base as builder

COPY . /catkin_ws/src/dr_hardware_tests
WORKDIR /catkin_ws
RUN /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/tmp/ros/noetic

#
# FINAL STAGE
FROM base

COPY --from=geoid-downloader /usr/share/GeographicLib/geoids /usr/share/GeographicLib/geoids
COPY --from=builder /tmp/ros/noetic /opt/ros/noetic


# CMD ["/ros_entrypoint.sh", "bash"]

#install pymavlink
# RUN pip3 install cython==0.29.24
# RUN pip3 install PyGeodesy

# COPY ./catkin_ws /catkin_ws
# WORKDIR /catkin_ws
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'
# RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

