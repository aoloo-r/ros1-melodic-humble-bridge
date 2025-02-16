# Use the ROS2 Humble base on Ubuntu 22.04
FROM ros:humble-ros-base-jammy

# Use Bash with better error handling
SHELL ["/bin/bash", "-o", "pipefail", "-o", "errexit", "-c"]

###########################
# 1.) Bring system up to the latest ROS2 desktop (Humble)
###########################
RUN apt-get -y update
RUN apt-get -y upgrade
RUN apt-get -y install ros-humble-desktop

###########################
# 2.) Temporarily remove ROS2 apt repository
###########################
RUN mv /etc/apt/sources.list.d/ros2-latest.list /root/
RUN apt-get update

###########################
# 3.) Comment out the catkin conflict in dpkg status
###########################
RUN sed -i -e 's|^Conflicts: catkin|#Conflicts: catkin|' /var/lib/dpkg/status
RUN apt-get install -f

###########################
# 4.) Force-install these Python packages
###########################
RUN apt-get download python3-catkin-pkg
RUN apt-get download python3-rospkg
RUN apt-get download python3-rosdistro
RUN dpkg --force-overwrite -i python3-catkin-pkg*.deb
RUN dpkg --force-overwrite -i python3-rospkg*.deb
RUN dpkg --force-overwrite -i python3-rosdistro*.deb
RUN apt-get install -f

###########################
# 5.) Install partial ROS1 environment from Ubuntu (ros-desktop-dev)
#    This is effectively a "Melodic-ish" snapshot on Ubuntu 22.04
###########################
RUN apt-get -y install ros-desktop-dev

# Fix ARM64 pkgconfig path issue
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/; \
    fi

###########################
# 6.) Restore the ROS2 apt repos and define build args
###########################
RUN mv /root/ros2-latest.list /etc/apt/sources.list.d/
RUN apt-get -y update

ARG ADD_ros_tutorials=1
ARG ADD_grid_map=0
ARG ADD_example_custom_msgs=0

RUN echo "ADD_ros_tutorials         = '$ADD_ros_tutorials'"
RUN echo "ADD_grid_map              = '$ADD_grid_map'"
RUN echo "ADD_example_custom_msgs   = '$ADD_example_custom_msgs'"

###########################
# 6.1) Add additional ros_tutorials messages and services
###########################
RUN if [[ "$ADD_ros_tutorials" = "1" ]]; then \
      git clone https://github.com/ros/ros_tutorials.git; \
      cd ros_tutorials; \
      git checkout melodic-devel; \
      # Patch the code to use boost::placeholders::_1
      sed -i 's/\b_1\b/boost::placeholders::_1/g' roscpp_tutorials/listener_with_userdata/listener_with_userdata.cpp; \
      unset ROS_DISTRO; \
      time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    fi

# unit test (optional)
RUN if [[ "$ADD_ros_tutorials" = "1" ]]; then \
      cd ros_tutorials; \
      unset ROS_DISTRO; \
      colcon test --event-handlers console_direct+; \
      colcon test-result; \
    fi


###########################
# 6.2) (Optional) Build grid_map from melodic-devel branches
###########################
# Navigation partial
RUN if [[ "$ADD_grid_map" = "1" ]]; then \
      apt-get -y install libsdl1.2-dev libsdl-image1.2-dev; \
      git clone https://github.com/ros-planning/navigation.git; \
      cd navigation; \
      git checkout melodic-devel; \
      unset ROS_DISTRO; \
      time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select map_server voxel_grid costmap_2d; \
    fi

# Filters
RUN if [[ "$ADD_grid_map" = "1" ]]; then \
      git clone https://github.com/ros/filters.git; \
      cd filters; \
      git checkout melodic-devel; \
      unset ROS_DISTRO; \
      time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    fi

# Grid_map
RUN if [[ "$ADD_grid_map" = "1" ]]; then \
      apt-get -y install libpcl-ros-dev libcv-bridge-dev; \
      source navigation/install/setup.bash; \
      source filters/install/setup.bash; \
      git clone https://github.com/ANYbotics/grid_map.git; \
      cd grid_map; \
      git checkout 1.6.4; \
      unset ROS_DISTRO; \
      grep -r c++11 | grep CMakeLists | cut -f 1 -d ':' \
        | xargs sed -i -e 's|std=c++11|std=c++17|g'; \
      time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-select grid_map_msgs grid_map_core grid_map_octomap grid_map_sdf \
                         grid_map_costmap_2d grid_map_cv grid_map_ros grid_map_loader; \
    fi

############################
# 6.3) Custom messages example
############################
RUN if [[ "$ADD_example_custom_msgs" = "1" ]]; then \
      git clone https://github.com/TommyChangUMD/custom_msgs.git; \
      # Compile ROS1 (melodic-ish)
      cd /custom_msgs/custom_msgs_ros1; \
      unset ROS_DISTRO; \
      time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
      # Compile ROS2 side
      cd /custom_msgs/custom_msgs_ros2; \
      source /opt/ros/humble/setup.bash; \
      time colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    fi

###########################
# 7.) Compile ros1_bridge for Melodic ↔ Humble
###########################
RUN \
    source /opt/ros/humble/setup.bash; \
    \
    if [[ "$ADD_ros_tutorials" = "1" ]]; then \
      source ros_tutorials/install/setup.bash; \
      apt-get -y install ros-humble-example-interfaces; \
      source /opt/ros/humble/setup.bash; \
    fi; \
    if [[ "$ADD_grid_map" = "1" ]]; then \
      source grid_map/install/setup.bash; \
      apt-get -y install ros-humble-grid-map; \
      source /opt/ros/humble/setup.bash; \
    fi; \
    if [[ "$ADD_example_custom_msgs" = "1" ]]; then \
      source /custom_msgs/custom_msgs_ros1/install/setup.bash; \
      source /custom_msgs/custom_msgs_ros2/install/setup.bash; \
    fi; \
    \
    mkdir -p /ros-humble-ros1-bridge/src; \
    cd /ros-humble-ros1-bridge/src; \
    git clone https://github.com/smith-doug/ros1_bridge.git; \
    cd ros1_bridge/; \
    git checkout action_bridge_humble; \
    cd ../..; \
    MEMG=$(printf "%.0f" $(free -g | awk '/^Mem:/{print $2}')); \
    NPROC=$(nproc); MIN=$((MEMG<NPROC ? MEMG : NPROC)); \
    echo "Please wait... running $MIN concurrent jobs to build ros1_bridge"; \
    time MAKEFLAGS="-j $MIN" colcon build --event-handlers console_direct+ \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

###########################
# 8.) Clean up
###########################
RUN apt-get -y clean all; apt-get -y update

###########################
# 9.) Pack all ROS1 dependent libraries
###########################
RUN if [[ $(uname -m) = "arm64" || $(uname -m) = "aarch64" ]]; then \
      cp /usr/lib/x86_64-linux-gnu/pkgconfig/* /usr/lib/aarch64-linux-gnu/pkgconfig/; \
    fi

RUN ROS1_LIBS="libxmlrpcpp.so"; \
    ROS1_LIBS="$ROS1_LIBS librostime.so"; \
    ROS1_LIBS="$ROS1_LIBS libroscpp.so"; \
    ROS1_LIBS="$ROS1_LIBS libroscpp_serialization.so"; \
    ROS1_LIBS="$ROS1_LIBS librosconsole.so"; \
    ROS1_LIBS="$ROS1_LIBS librosconsole_log4cxx.so"; \
    ROS1_LIBS="$ROS1_LIBS librosconsole_backend_interface.so"; \
    ROS1_LIBS="$ROS1_LIBS liblog4cxx.so"; \
    ROS1_LIBS="$ROS1_LIBS libcpp_common.so"; \
    ROS1_LIBS="$ROS1_LIBS libb64.so"; \
    ROS1_LIBS="$ROS1_LIBS libaprutil-1.so"; \
    ROS1_LIBS="$ROS1_LIBS libapr-1.so"; \
    ROS1_LIBS="$ROS1_LIBS libactionlib.so.1d"; \
    cd /ros-humble-ros1-bridge/install/ros1_bridge/lib; \
    for soFile in $ROS1_LIBS; do \
      soFilePath=$(ldd libros1_bridge.so | grep $soFile | awk '{print $3;}'); \
      cp $soFilePath ./; \
    done

###########################
# 10.) Output ros1_bridge tarball by default
###########################
RUN tar czf /ros-humble-ros1-bridge.tgz \
    --exclude '*/build/*' --exclude '*/src/*' /ros-humble-ros1-bridge

ENTRYPOINT []
CMD cat /ros-humble-ros1-bridge.tgz; sync
