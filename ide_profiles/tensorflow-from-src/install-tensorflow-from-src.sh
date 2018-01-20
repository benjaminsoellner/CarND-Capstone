#!/bin/bash

cd /var/team-herbie

# do all tensorflow-from-source stuff in ~/tensorflow-src
mkdir tensorflow-src
cd tensorflow-src

# Install Bazel
wget https://github.com/bazelbuild/bazel/releases/download/0.5.1/bazel-0.5.1-installer-linux-x86_64.sh
chmod u+x bazel-0.5.1-installer-linux-x86_64.sh
sudo ./bazel-0.5.1-installer-linux-x86_64.sh

# Install python dependencies
sudo apt-get install python-numpy python-dev python-pip python-wheel

# Clone github repo
git clone https://github.com/tensorflow/tensorflow
cd tensorflow
git checkout r1.3

# Configure installation
# see: https://www.tensorflow.org/versions/r1.5/install/install_sources#ConfigureInstallation
./configure

# Build Tensorflow
#sudo bazel build --config=opt //tensorflow/tools/pip_package:build_pip_package

cd -
