# Modify Users & Groups
```
sudo groupadd herbie
sudo usermod -a -G herbie student
sudo usermod -g herbie student
sudo mkdir /var/team-herbie
sudo chown student:herbie /var/team-herbie/ -R
```

# Add Project Users
Use `/ide_profiles/azure-vm/copy_user.sh` script.

# Setup CarND Repo
```
sudo mkdir /var/team-herbie
cd /var/team-herbie/
git clone https://github.com/team-herbie/CarND-Capstone.git
chown student:herbie /var/team-herbie -R
chmod g+w /var/team-herbie -R
cd /var/team-herbie/CarND-Capstone
```

# Install ROS
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

# Install CarND Dependencies
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

# xrdp + kde
```
# see http://c-nergy.be/blog/?p=6717
# for lubuntu (classroom VM):
#   * https://wademurray.com/2014/xrdp-remote-desktop-on-lubuntu-14-04/
#   * https://wiki.ubuntu.com/Lubuntu/RemoteDesktop
#   * https://sourceforge.net/p/guacamole/discussion/1110833/thread/cb16f8b5/
```

# guacamole
```
# https://www.chasewright.com/guacamole-with-mysql-on-ubuntu/
```
