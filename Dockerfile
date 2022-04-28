#
# GEOID DOWNLOADER
FROM ubuntu:20.04 as geoid-downloader

RUN apt-get update && \
    apt-get install -y geographiclib-tools && \
    geographiclib-get-geoids egm96-5 

#
# BUILDER BASE
FROM ros:noetic-ros-base-focal AS base

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


CMD ["/ros_entrypoint.sh", "/bin/bash"]

#install pymavlink
# RUN pip3 install cython==0.29.24
# RUN pip3 install PyGeodesy

# COPY ./catkin_ws /catkin_ws
# WORKDIR /catkin_ws
# RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'
# RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

