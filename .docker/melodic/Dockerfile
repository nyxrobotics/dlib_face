# Sets up a base image to use for running CI on Travis
FROM ros:melodic-ros-base
LABEL maintainer="Tacha-S tacchan.mello.ioiq@gmail.com"

# Setup (temporary) ROS workspace
WORKDIR /opt/workspace

ENV PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig

RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -qq update && \
    apt-get -qqy dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -qq install --no-install-recommends -y \
    # Some basic requirements
    git sudo python-pip \
    # Preferred build tools
    python-catkin-tools clang clang-format-10 clang-tidy-10 clang-tools ccache ros-melodic-roslint && \
    pip install compdb catkin-lint && \
    #
    # build adn install requirements
    git clone --depth 1 https://github.com/davisking/dlib.git && \
    mkdir dlib/build && cd dlib/build && cmake -DBUILD_SHARED_LIBS=ON .. && cmake --build . --config Release && \
    make install && cd /opt/workspace && rm -rf dlib && \
    # Download source
    git clone --depth 1 -b main https://github.com/Tacha-S/dlib_face.git src/dlib_face && \
    #
    # Download all dependencies
    rosdep update && \
    rosdep install -y --from-paths src -i --rosdistro ${ROS_DISTRO} && \
    # Remove the source code from this container
    rm -rf src && \
    #
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/*
