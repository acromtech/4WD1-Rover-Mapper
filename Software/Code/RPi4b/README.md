# Raspberry Pi code powered by ROS
## Description
* `main.cpp` robot initialisations and process (manual/automatic)
* `Robot.h` check or change sensors/actuators (wiringPi) pinout

## Prerequisite
**Ubuntu 22.04 LTS on your Laptop/Desktop**
> To install [Ubuntu as dualboot] (https://lecrabeinfo.net/installer-ubuntu-22-04-lts-en-dual-boot-avec-windows.html)(with Windows 11).

**Ubuntu 22.04 LTS on your RaspberryPi**
> To install Ubuntu you can use [RaspberryPi Imager](https://www.raspberrypi.com/software/)

## Installation
### Open-ssh installation
On your `RaspberryPi` open the terminal with `Ctrl+Alt+T` and install the openssh-server package:
```
sudo apt update
sudo apt install openssh-server
```

Once the installation is complete, the SSH service will start automatically. You can verify that SSH is running by typing:
```
sudo systemctl status ssh
```

The output should tell you that the service is running and enabled to start on system boot:
```
● ssh.service - OpenBSD Secure Shell server
    Loaded: loaded (/lib/systemd/system/ssh.service; enabled; vendor preset: enabled)
    Active: active (running) since Mon 2020-06-01 12:34:00 CEST; 9h ago
...
```

Ubuntu ships with a firewall configuration tool called UFW. If the firewall is enabled on your system, make sure to open the SSH port:
```
sudo ufw allow ssh
```
### Launch SSH connection
To connect to your Ubuntu machine over LAN invoke the ssh command followed by the username and the IP address in the following format:
```
ssh username@ip_address
```
> If you don’t know your IP address you can easily find it using the ip command :
> ```
> ip a
> ```

### ROS 2 Humble Hawksbill installation
Install [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your **RaspberryPi and your Laptop/Desktop**
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
# 
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

> Test your installation
> ```
> # In one terminal (connect to your RaspabrryPi in SSH and) launch the talker program
> source /opt/ros/humble/setup.bash
> ros2 run demo_nodes_cpp talker
> 
> # In an other terminal on your Laptop and launch the listener program
> source /opt/ros/humble/setup.bash
> ros2 run demo_nodes_py listener
> ```
> Output
> ```
> # On RaspberryPi
> ...
> [INFO] [1685997570.172011767] [talker]: Publishing: 'Hello World: 10'
> [INFO] [1685997571.171814502] [talker]: Publishing: 'Hello World: 11'
> [INFO] [1685997572.171789659] [talker]: Publishing: 'Hello World: 12'
> [INFO] [1685997573.171785389] [talker]: Publishing: 'Hello World: 13'
> [INFO] [1685997574.171754769] [talker]: Publishing: 'Hello World: 14'
> ...
> 
> # On Laptop/Desktop
> [INFO] [1685997579.213332816] [listener]: I heard: [Hello World: 10]
> [INFO] [1685997580.226675399] [listener]: I heard: [Hello World: 11]
> [INFO] [1685997581.251356690] [listener]: I heard: [Hello World: 12]
> [INFO] [1685997582.274788642] [listener]: I heard: [Hello World: 13]
> [INFO] [1685997583.298012730] [listener]: I heard: [Hello World: 14]
> ...
> ```

Install [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) on your **RaspberryPi only**
```
sudo apt install python3-colcon-common-extensions
```
Configure your [ROS 2 Humble Hawksbill environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) on your RaspberryPi and your Laptop/Desktop
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
> Re-open 2 new terminal and verify the ROS 2 Humble Hawksbill environnement
> ```
> # In one terminal
> ros2 run demo_nodes_cpp talker
> 
> # In an other terminal 
> ros2 run demo_nodes_py listener
> ```

*Check the [Articulated Robotics](https://www.youtube.com/watch?v=uWzOk0nkTcI) video for more installation informations.*

*WARNING : The video show an older version of ROS installation.*
