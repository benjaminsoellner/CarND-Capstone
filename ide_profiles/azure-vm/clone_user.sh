#!/bin/bash

# Usage: ./clone-user.sh src_user_name new_user_name

SRC=$1
DEST=$2

SRC_GROUPS=$(id -Gn ${SRC} | sed "s/${SRC} //g" | sed "s/ ${SRC}//g" | sed "s/ /,/g")
SRC_SHELL=$(awk -F : -v name=${SRC} '(name == $1) { print $7 }' /etc/passwd)
useradd --groups ${SRC_GROUPS} --shell ${SRC_SHELL} --create-home ${DEST}
echo "source /opt/ros/kinetic/setup.bash" >> /home/${DEST}/.bashrc
passwd ${DEST}
