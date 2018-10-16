FROM ubuntu:14.04 

RUN apt-get update && \
    apt-get -y install curl wget

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && wget http://packages.ros.org/ros.key -O - | apt-key add -

RUN apt-get update
RUN apt-get install -y cmake python-catkin-tools python-empy python-pip python-dev python-matplotlib python-numpy python-scipy 
RUN apt-get install -y libeigen3-dev libboost-dev libboost-system-dev libboost-filesystem-dev libboost-program-options-dev libgtest-dev 
RUN apt-get install -y git 

RUN pip install --upgrade pip
RUN pip install ipython==5.5
