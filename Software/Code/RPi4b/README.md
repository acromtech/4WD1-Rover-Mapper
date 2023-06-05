# Code for Raspberry Pi
## Description
* main.cpp -> robot initialisations and process (manual/automatic)
* Robot.h -> Check or change sensors/actuators (wiringPi) pinout

## Installation on RaspberryPi
### Open-ssh

Open the terminal with Ctrl+Alt+T and install the openssh-server package:
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

### ROS
To install ROS check the [website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Installation on Laptop/Desktop
### Ubuntu (dual boot)
To install Ubuntu as dualboot (with Windows 11) check the well explain [lecrabeinfo tutorial](https://lecrabeinfo.net/installer-ubuntu-22-04-lts-en-dual-boot-avec-windows.html).

### ROS
Install the [same ROS installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) as your RaspberryPi on your Laptop/Desktop.
