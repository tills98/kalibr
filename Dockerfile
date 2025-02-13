# For more information, please refer to https://aka.ms/vscode-docker-python
FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# Keeps Python from generating .pyc files in the container
ENV PYTHONDONTWRITEBYTECODE=1

# Turns off buffering for easier container logging
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y \
	git wget autoconf automake nano \
	python3-dev python3-pip python3-scipy python3-matplotlib \
	ipython3 python3-wxgtk4.0 python3-tk python3-igraph python3-pyx \
	libeigen3-dev libboost-all-dev libsuitesparse-dev \
	doxygen \
	libopencv-dev \
	libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev \
	python3-catkin-tools python3-osrf-pycommon \
    gdb python3-dbg

RUN pip install numpy

ENV WORKSPACE /catkin_ws

RUN mkdir -p $WORKSPACE/src && \
	cd $WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/noetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug

WORKDIR /catkin_ws
COPY . $WORKSPACE/src/kalibr

RUN	cd $WORKSPACE &&\
	catkin build -j$(nproc --ignore 6)


# Creates a non-root user with an explicit UID and adds permission to access the /app folder
# For more info, please refer to https://aka.ms/vscode-docker-python-configure-containers
RUN adduser -u 5678 --disabled-password --gecos "" appuser && chown -R appuser /catkin_ws
USER appuser

# During debugging, this entry point will be overridden. For more information, please refer to https://aka.ms/vscode-docker-python-debug
CMD ["/bin/bash -c", "python", "aslam_offline_calibration/kalibr/python/kalibr_joint_calibration.py"]
