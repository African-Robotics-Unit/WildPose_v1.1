FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y

# Python
RUN apt-get install -y python3 python3-pip

# XIMEA
WORKDIR /tmp
COPY xiapi_parallels.sh /tmp/xiapi.sh
RUN chmod +x xiapi.sh
RUN ./xiapi.sh

# OpenCV
RUN apt-get install -y libopencv-dev python3-opencv

# PCL
RUN apt-get install -y libpcl-dev
RUN apt-get install -y python3-pcl pcl-tools
RUN python3 -m pip install -U setuptools
RUN python3 -m pip install python-pcl

# eCAL
RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:ecal/ecal-latest
RUN apt-get update
RUN apt-get install -y python3-ecal5
