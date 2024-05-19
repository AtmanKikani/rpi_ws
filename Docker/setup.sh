#!/bin/bash

sudo apt-get update
sudo apt-get upgrade -y

sudo apt-get install -y --no-install-recommends \
    apt-utils \
    nano \
    vim \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    libncurses5-dev \
    libncursesw5-dev \
    libeigen3-dev \
    libuvc-dev \
    git \
    build-essential \
    cmake \
    unzip \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libatlas-base-dev \
    gfortran \
    python3-pip \
    libxml2-dev \
    libxslt-dev \
    python3-numpy \
    python3-pytest \
    ros-noetic-libuvc-camera

cd /tmp
git clone --depth 1 --branch 4.6.0 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.6.0 https://github.com/opencv/opencv_contrib.git

cd /tmp/opencv
mkdir -p build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
      -D BUILD_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D WITH_CUDA=OFF \
      -D BUILD_OPENCV_PYTHON3=ON \
      -D BUILD_opencv_python3=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_DOCS=OFF \
      ..
make -j$(nproc)
sudo make install
sudo ldconfig

rm -rf /tmp/opencv /tmp/opencv_contrib

sudo python3 -m pip install --upgrade pip
sudo pip install pyserial
sudo python3 -m pip install --upgrade pymavlink

source /opt/ros/noetic/setup.bash

echo "All dependencies have been installed successfully."
