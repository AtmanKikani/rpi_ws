FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
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
    python3-pip

RUN pip3 install pyserial pymavlink

WORKDIR /tmp
RUN git clone --depth 1 --branch 4.6.0 https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch 4.6.0 https://github.com/opencv/opencv_contrib.git

WORKDIR /tmp/opencv
RUN mkdir -p build && \
    cd build && \
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
          .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

RUN rm -rf /tmp/opencv /tmp/opencv_contrib

RUN apt install ros-noetic-libuvc-camera

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/noetic/setup.bash 

CMD ["bash"]
