#!/bin/bash

if [ "$EUID" -ne 0 ]
  then 
  echo "###########################################################################################"
  echo ">>> PLEAS RUN THIS INSTALLATION SCRIPT AS ROOT <<<"
  echo "###########################################################################################"
  exit
fi

now=$(date +%d-%m-%Y" "%H:%M:%S | sed -e "s/ /_/g")



sudo apt update
apt -y upgrade 

# Installation Basis Software #########################################################

echo "###########################################################################################"
echo "Installation von Basis Software"
echo "###########################################################################################"

sudo apt install -y curl build-essential gdb gnupg2 lsb-release git 

# sudo apt install -y python-pip python-dev
sudo apt install -y python3 python3-pip
sudo apt install -y libssl-dev libffi-dev python-dev
sudo apt install -y portaudio19-dev


echo "###########################################################################################"
echo "Installation NodeJS"
echo "###########################################################################################"

# Using Ubuntu
# https://github.com/nodesource/distributions/blob/master/README.md#debinstall
curl -sL https://deb.nodesource.com/setup_15.x | sudo -E bash -
sudo apt-get install -y nodejs

# GCC if not installed for native addons
sudo apt-get install gcc g++ make

# Yarn for Node JS
curl -sL https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt-get update && sudo apt-get install yarn

echo "###########################################################################################"
echo "Installation NGINX"
echo "###########################################################################################"
sudo apt install nginx


echo "###########################################################################################"
echo "Installation ROS2"
echo "###########################################################################################"

# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-foxy-desktop
# sudo apt install ros-foxy-ros-base #ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools


# Installation Redis #########################################################

read -p "Redis..." answer

echo "###########################################################################################"
echo "Installation Redis Server"
echo "See https://phoenixnap.com/kb/install-redis-on-ubuntu-20-04"
echo "###########################################################################################"

apt install -y redis-server

echo ">>>>>>>>> Setup Redis Config"

cp /etc/redis/redis.conf /etc/redis/redis${now}.confBackup
sed -i 's/supervised no/supervised systemd/' /etc/redis/redis.conf

systemctl restart redis.service
sleep 1

echo ">>>>>>>>> Check Redis:"

systemctl status redis
