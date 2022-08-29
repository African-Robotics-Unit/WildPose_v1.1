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

#### dotfiles

Set the dotfiles you wanna use ([Naoya's dotfiles](https://github.com/DenDen047/dotfiles)).

#### Network Configuration

On the Jetson, you can use `nmcli` command to change the network settings.

Set the static IP on the UCT network ([reference](https://f1tenth.readthedocs.io/en/stable/getting_started/software_setup/optional_software_nx.html)).
```bash
$ nmcli c show
NAME                UUID                                  TYPE      DEVICE
Wired connection 1  b72f3d20-4de2-3d44-9c45-9689d79f22e4  ethernet  eth0
docker0             bcb6f95d-5cf5-483d-ac09-c312a4da8c0b  bridge    docker0
$ sudo nmcli c mod "Wired connection 1" ipv4.address [NEW_ADDRESS]/27
```

#### ROS2 Foxy

Let's install [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) following with [the official guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
See `setup_scripts/ros2_foxy.sh`.

#### The Official Ximea Camera Driver

This is [the original GitHub repository](https://github.com/wavelab/ximea_ros_cam).

##### 1. Install Xiema Software Package.

```bash
$ cd ~/Downloads
$ mkdir tmp
$ cd ~/Downloads/tmp
$ wget https://www.ximea.com/support/attachments/download/271/XIMEA_Linux_SP.tgz
$ tar -xf XIMEA_Linux_SP.tgz
$ cd ~/Downloads/tmp/package
$ ./install -cam_usb30
$ cd ~/Downloads
$ rm -rf tmp
```

##### 2. Add user to the `plugdev` group

```bash
$ sudo gpasswd -a $USER plugdev
```

##### 3. Setup the USB FS Memory Max Allocation to Infinite

This is done to make sure that the USB FS buffering size is sufficient for high bandwidth streams through USB 3.0

*Set this with every new shell*:
Put `echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb` into `/etc/rc.local`

Or

*Apply to current shell*:
`echo "0" | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`

##### 4. Set realtime priority to the `/etc/security/limits.conf`

Place the following in `/etc/security/limits.conf` to make the Ximea camera driver have real time priority.

```
*               -       rtprio          0
@realtime       -       rtprio          81
*               -       nice            0
@realtime       -       nice            -16
```

Then add the current user to the group `realtime`:
```bash
$ sudo groupadd realtime
$ sudo gpasswd -a $USER realtime
```

#### M2S2 for ximea camera driver

```bash
$ cd ~/ros2_ws
$ git clone https://github.com/African-Robotics-Unit/M2S2.git
$ cd ~/ros2_ws/M2S2
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
$ make
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

Update the config file.

### Host Computer

### Visual Studio Code

https://youtu.be/VeOj_C6rAF4?t=327
