#!/bin/bash

# This script is based on [the official guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

# Set locale
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
