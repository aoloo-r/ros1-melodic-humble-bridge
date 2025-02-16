# ROS1 (Melodic) ↔ ROS2 (Humble) Bridge Docker Setup

This repository provides a Dockerfile and instructions for building a Docker image that contains a ROS2 Humble environment along with a partial ROS1 (Melodic) environment. The image is used to build and package the `ros1_bridge` so that topics can be bridged between a ROS1 (Melodic) robot and a ROS2 (Humble) desktop.

This setup is especially useful for remote control and integration where your robot runs ROS1 (Melodic) (e.g., on Ubuntu 18.04) and your powerful desktop (e.g., with an RTX 4090 on Ubuntu 22.04) runs ROS2 Humble.

## Overview

- **Docker Image:** Based on Ubuntu 22.04, it installs ROS2 Humble (desktop) via apt and a "Melodic-ish" ROS1 environment (using `ros-desktop-dev` from Ubuntu 22.04) plus additional packages (e.g., `ros_tutorials`) to support a ROS1 (Melodic) environment.
- **Bridge Build:** The image builds the `ros1_bridge` (Melodic ↔ Humble) using colcon.
- **Usage:** After building, you extract a precompiled tarball (the bridge overlay) that can be sourced on your desktop to run the dynamic bridge.

## Prerequisites

- A desktop running Ubuntu 22.04 with ROS2 Humble installed.
- A robot running Ubuntu 18.04 with ROS1 Melodic.
- Docker installed on your desktop.
- Both devices must be on the same network (or connected via Ethernet) with routable IP addresses.

For example, in our setup:
- **Robot IP:** `10.66.171.191`
- **Desktop IP:** `10.66.171.108`

## Building the Docker Image

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/aoloo-r/ros1-melodic-humble-bridge.git
   cd ros1-melodic-humble-bridge

2. **Build the Docker Image:**
   ```bash
   docker build -t ros-humble-ros1-bridge-builder .
   
Note: This build may take a while as it compiles parts of ROS1 from source and builds the bridge.  

## Part 2: Extract the Bridge Overlay
After the image is built, run the following command to extract the precompiled bridge overlay tarball:

```bash
docker run --rm ros-humble-ros1-bridge-builder | tar xzvf -

This creates a directory named ros-humble-ros1-bridge in your current folder.

