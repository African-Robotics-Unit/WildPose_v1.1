FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y

# Python
RUN apt-get install -y python3 python3-pip
RUN python3 -m pip install -U setuptools ipython

# OpenCV
RUN apt-get install -y libopencv-dev python3-opencv

# XIMEA
RUN apt install -y ca-certificates wget sudo vim
RUN python3 -m pip install matplotlib Pillow numpy
WORKDIR /tmp
RUN wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
RUN tar -xf XIMEA_Linux_SP.tgz && rm XIMEA_Linux_SP.tgz
WORKDIR /tmp/package
RUN ./install -noudev
RUN python3 -m pip install ximea-py

# PCL
# RUN apt-get install -y libpcl-dev
# RUN apt-get install -y python3-pcl pcl-tools
# RUN python3 -m pip install python-pcl

# # eCAL
# RUN apt-get install -y software-properties-common
# RUN add-apt-repository ppa:ecal/ecal-latest
# RUN apt-get update
# RUN apt-get install -y python3-ecal5

# # Other pkgs
# RUN python3 -m pip install ipython
