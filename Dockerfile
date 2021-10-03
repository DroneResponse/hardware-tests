FROM ros:noetic-ros-base-focal

# install python packages
RUN apt update && apt install --yes \
    python-is-python3 \
    python3 \
    python3-pip \
    python3-venv \
    python3-wheel \
    python3-dev \   
    python3-catkin-tools

# install mavros
RUN apt install --yes \
    ros-noetic-mavros \
    ros-noetic-mavros-extras 

RUN apt install --yes vim

#install pymavlink

RUN pip3 install pymavlink
RUN pip3 install cython==0.29.24
RUN pip3 install PyGeodesy
RUN geographiclib-get-geoids egm96-5 
RUN pip3 install nvector==0.7.6



RUN pip3 install http://docs.q3w.co/droneresponse_mathtools-0.2.1-py2.py3-none-any.whl

# clean up
RUN rm -rf /var/lib/apt/lists/*

COPY ./catkin_ws /catkin_ws

WORKDIR /catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

