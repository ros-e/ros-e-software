#!/bin/bash

if [ "$EUID" -ne 0 ]
  then 
  echo "###########################################################################################"
  echo ">>> PLEAS RUN THIS INSTALLATION SCRIPT AS ROOT <<<"
  echo "###########################################################################################"
  exit
fi

now=$(date +%d-%m-%Y" "%H:%M:%S | sed -e "s/ /_/g")



apt update
apt -y upgrade 

# Installation Basis Software #########################################################

echo "###########################################################################################"
echo "Installation von Basis Software"
echo "###########################################################################################"

apt install -y curl build-essential gdb gnupg2 lsb-release git 

# sudo apt install -y python-pip python-dev
apt install -y python3 python3-pip
apt install -y libssl-dev libffi-dev python-dev
apt install -y portaudio19-dev


pythonPackages=(redis numpy scipy nose pyusb pyaudio pixel-ring argcomplete smbus) #opencv-python matplotlib ipython jupyter pandas sympy

for package in "${pythonPackages[@]}"
  do
    echo ">>>>>>>>> Install $package"
    python3 -m pip install "$package"
done

echo "###########################################################################################"
echo "Installation NGINX"
echo "###########################################################################################"
apt install -y nginx

# Setup nginx configuration
cp /etc/nginx/sites-enabled/default /etc/nginx/sites-enabled/default_${now}.backup
cp -p /home/rose/software/setup/nginx-config.txt /etc/nginx/sites-enabled/default
systemctl restart nginx


echo "###########################################################################################"
echo "Installation NodeJS"
echo "###########################################################################################"

# Using Ubuntu
# https://github.com/nodesource/distributions/blob/master/README.md#debinstall
curl -sL https://deb.nodesource.com/setup_15.x | -E bash - # For v15
# curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -  # You need 12 for if you want to use ROS2 NodeJS https://github.com/RobotWebTools/rclnodejs
apt-get install -y nodejs

# GCC if not installed for native addons
apt-get install -y gcc g++ make

# Yarn for Node JS
curl -sL https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list
apt-get update && sudo apt-get install yarn

# Install node process manager
npm install pm2 -g

# Run node applications
env PATH=$PATH:/usr/bin /usr/lib/node_modules/pm2/bin/pm2 startup systemd -u rose --hp /home/rose
sudo -u rose bash /home/rose/software/node/init.sh

echo "###########################################################################################"
echo "Installation ROS2"
echo "###########################################################################################"

# https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
apt update
# sudo apt install ros-foxy-desktop
apt install -y ros-foxy-ros-base #ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools
apt install -y python3-colcon-common-extensions

sudo -u rose source /opt/ros/foxy/setup.bash
sudo -u rose echo "source /opt/ros/foxy/setup.bash" >> /home/rose/.bashrc

# Creating goto script for easier ROS2 Setup in home directory
sudo -u rose echo "source ~/software/ros2/ros_ws/install/setup.bash" > /home/rose/goto.sh
sudo -u rose echo "cd ~/software/ros2/ros_ws" >> /home/rose/goto.sh
sudo -u rose chmod u+x /home/rose/goto.sh

# Build ROS2
cd /home/rose/software/ros2/ros_ws/ && sudo -u rose colcon build


# Installation Redis #########################################################


# read -p "Redis..." answer

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




echo "###########################################################################################"
echo "Configure I2C"
echo "###########################################################################################"

apt-get install -y python3-smbus
apt-get install -y i2c-tools
apt install -y libi2c-dev

# RasPi Config
# https://askubuntu.com/questions/1273700/enable-spi-and-i2c-on-ubuntu-20-04-raspberry-pi
wget https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20201108_all.deb -P /tmp
dpkg -i /tmp/raspi-config_20201108_all.deb
apt-get install -y lua5.1
apt-get install -y alsa-utils
apt --fix-broken install
dpkg -i /tmp/raspi-config_20201108_all.deb
raspi-config
# Run raspi-config and enable I2C 

groupadd i2c
chown :i2c /dev/i2c-1
chmod g+rw /dev/i2c-1
usermod -aG i2c rose
