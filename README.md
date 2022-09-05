# WildPose v1.1

## Hardware

### XIMEA -- MQ022CG-CM

![Slide86](https://user-images.githubusercontent.com/6120047/187175093-c170c1db-6820-45db-b62d-7cf7d2296982.jpeg)

## Prerequisite

- JetPack v5
- ROS2 Foxy

## Setup

### Jetson AGX Xavier Developer Kit

#### JetPack

We used a [Jetson AGX Xavier Developer Kit](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit).
To set up the Jetson AGX Xavier Developer Kit, we need a host computer installed Ubuntu 20.04.

1. Download and install [NVIDIA SDK Manager](https://developer.nvidia.com/nvidia-sdk-manager) onto the host computer.
2. Connect the host computer and Jetson AGX Xavier Developer Kit (see this [video](https://www.youtube.com/watch?v=-nX8eD7FusQ)).
3. Run NVIDIA SDK Manager and follow the instruction.

Unleash the limitation of CPU ([the reference](https://forums.developer.nvidia.com/t/cpus-usage-problem-solved/65993/3)).
```bash
$ sudo nvpmodel -m 0
$ sudo /usr/bin/jetson_clocks
```

#### General Settings

- Settings > Power > Blank Screen > `Never`

#### Network Configuration

On the Jetson, you can use `nmcli` command to change the network settings.

Set the static IP on the UCT network ([reference](https://f1tenth.readthedocs.io/en/stable/getting_started/software_setup/optional_software_nx.html)).
```bash
$ nmcli c show
NAME                UUID                                  TYPE      DEVICE
Wired connection 1  b72f3d20-4de2-3d44-9c45-9689d79f22e4  ethernet  eth0
docker0             bcb6f95d-5cf5-483d-ac09-c312a4da8c0b  bridge    docker0
$ sudo nmcli c modify "Wired connection 1" ipv4.address [NEW_ADDRESS]/27
```

#### SSH

***HOST COMPUTER***

For the ssh login from your computer, you should make a pair of ssh key on the host computer.
```bash
$ ssh-keygen -t rsa
```

Then, copy the public key into the Jetson.
```bash
$ ssh-copy-id -i ~/.ssh/wildpose_jetsonagx.pub [user]@[ip address]
```

Add the jetson IP address information in `~/.ssh/config`:
```
Host [ip address]
    HostName [ip address]
    User naoya
    IdentityFile ~/.ssh/wildpose_jetsonagx
    UseKeychain yes
    AddKeysToAgent yes
```

#### dotfiles

Set the dotfiles you wanna use (e.g., [Naoya's dotfiles](https://github.com/DenDen047/dotfiles)).

#### ROS2 Foxy

Let's install [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) following with [the official guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
See `setup_scripts/ros2_foxy.sh`.

#### The Official Ximea Camera Driver

This is [the original GitHub repository](https://github.com/wavelab/ximea_ros_cam) and [the Guide for Jetson](https://www.ximea.com/support/wiki/apis/Linux_TX1_and_TX2_Support#Installing-XIMEA-API-package).


```bash
$ cd setup_scripts/
$ chmod +x ./xiapi.sh
$ ./xiapi.sh
```

##### Setup the USB FS Memory Max Allocation to Infinite

This is done to make sure that the USB FS buffering size is sufficient for high bandwidth streams through USB 3.0

*Set this with every new shell*:
Put `echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb` into `/etc/rc.local`

Or

*Apply to current shell*:
`echo "0" | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`


#### M2S2 for ximea camera driver

```bash
$ cd ~/ros2_ws
$ git clone git@github.com:African-Robotics-Unit/M2S2.git
$ cd ~/ros2_ws/M2S2
$ git fetch
$ git checkout -b ros-drivers
$ sudo apt install -y ros-foxy-camera-info-manager
$ colcon build --packages-select ximea_ros2_cam
```

Then, add `source ~/ros2_ws/M2S2/install/setup.bash` into `~/.bashrc`.

To avoid the [error 45](https://github.com/Fu-physics/Ximea/blob/master/xiPython/v3/ximea/xidefs.py#L49), you have to run the following command.

```bash
$ sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0
```

#### image_view

To show the ximea camera image data, you are recommended to install [image_view](https://index.ros.org/p/image_view/).

```bash
$ sudo apt install -y ros-foxy-image-view
```

#### Livox-SDK

This is [the original GitHub repository](https://github.com/Livox-SDK/Livox-SDK).

```bash
$ sudo apt install -y cmake
$ cd ~/Documents
$ git clone https://github.com/Livox-SDK/Livox-SDK.git
$ cd Livox-SDK
$ cd build && cmake ..
$ make EXTRA_CXXFLAGS=-fPIC
$ sudo make install
```

#### Livox ROS2 Driver

This is [the original GitHub repository](https://github.com/Livox-SDK/livox_ros2_driver).

```bash
$ cd ~/ros2_ws
$ git clone https://github.com/Livox-SDK/livox_ros2_driver.git livox_ros2_driver/src
$ cd livox_ros2_driver
$ colcon build
$ source ~/ros2_ws/livox_ros2_driver/install/setup.bash
```

Add `source ~/ros2_ws/livox_ros2_driver/install/setup.bash` into `~/.bashrc`.
Don't forget to change **the config file**.

### Host Computer

To develop ROS2 programs on your host/local computer, VS Code ROS Extension was used.
Please refer to see the following video:

<iframe width="560" height="315" src="https://www.youtube.com/embed/teA20AjBlG8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
