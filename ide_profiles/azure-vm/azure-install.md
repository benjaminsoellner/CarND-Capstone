## VM Image and Sizing

Choose this virtual image:

![The Microsoft Data Science VM for Linux (Ubuntu)](data-science-vm.png)

Choose this VM size:

![Chosen Size for the VM: ND6S](vm-size.png)

## Remove Clutter
```
sudo rm -rf /anaconda
#sudo rm -rf /opt/hadoop
#sudo rm -rf /opt/microsoft/azureml/
#sudo rm /etc/init.d/tomcat7
```

## Modify Users & Groups
```
sudo groupadd herbie
sudo usermod -a -G herbie student
sudo usermod -g herbie student
sudo mkdir /var/team-herbie
sudo chown student:herbie /var/team-herbie/ -R
```

## Install X11RDP + RDP
Do not forget to set a `passwd`:
```
sudo passwd student
```

```
sudo apt-get update
sudo apt-get install tightvncserver -y
sudo apt-get install xrdp -y
sudo apt-get install xfce4
echo xfce4-session >~/.xsession
sudo groupadd tsusers
sudo usermod student -a -g tsusers
sudo sed -i.bak '/fi/a #xrdp multiple users configuration \n xfce4-session \n' /etc/xrdp/startwm.sh
sudo service xrdp restart


cd $HOME
mkdir install
cd install
git clone https://github.com/scarygliders/X11RDP-o-Matic.git
cd X11RDP-o-Matic
sudo ./X11rdp-o-matic.sh --nocpuoptimize
sudo ./RDPsesconfig.sh
```

## Install Guacamole
SKIP THIS - THIS DOES NOT WORK!
```
cd $HOME/install
wget https://raw.githubusercontent.com/MysticRyuujin/guac-install/master/guac-install.sh
chmod +x guac-install.sh
./guac-install.sh
```

## Setup CarND Repo
```
cd /var/team-herbie/
umask 002
sudo chgrp herbie .
sudo chmod g+ws .
git clone https://github.com/team-herbie/CarND-Capstone.git
cd /var/team-herbie/CarND-Capstone
```

## Install ROS
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update -y
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init -y
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
```

## Install CarND Dependencies
```
sudo sh -c 'echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-dataspeed-public.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys FF6D3CDA
sudo apt-get update -y
sudo sh -c 'echo "yaml http://packages.dataspeedinc.com/ros/ros-public-'$ROS_DISTRO'.yaml '$ROS_DISTRO'" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-'$ROS_DISTRO'.list'
rosdep update
sudo apt-get install -y ros-$ROS_DISTRO-dbw-mkz -y
sudo apt-get upgrade -y
sudo apt-get install -y python-pip
pip install -r requirements-gpu.txt
sudo apt-get install -y ros-$ROS_DISTRO-cv-bridge
sudo apt-get install -y ros-$ROS_DISTRO-pcl-ros
sudo apt-get install -y ros-$ROS_DISTRO-image-proc
sudo apt-get install -y netbase
```

## Install tensorflow-from-src

Use `/ide_profiles/tensorflow-from-src/install-tensorflow-from-src.sh` script.

## Install Simulator

```
cd /var/team-herbie
wget https://github.com/udacity/CarND-Capstone/releases/download/v1.3/linux_sys_int.zip
```


## Add Project Users
Use `/ide_profiles/azure-vm/copy_user.sh` script.
