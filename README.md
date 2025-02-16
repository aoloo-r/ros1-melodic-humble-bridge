Below is the complete README content wrapped in a code snippet. Simply copy and paste the following into your `README.md` file:

```markdown
# ROS1 (Melodic) ↔ ROS2 (Humble) Bridge Docker Setup

This repository provides a Dockerfile and instructions for building a Docker image that creates a ROS2 Humble environment along with a partial ROS1 (Melodic) environment. The image is used to build and package the `ros1_bridge` so that topics can be bridged between a ROS1 (Melodic) robot and a ROS2 (Humble) desktop.

This setup is especially useful if your robot runs ROS1 (Melodic) on Ubuntu 18.04 and your desktop (e.g., with an RTX 4090) runs ROS2 Humble on Ubuntu 22.04. In our example, we assume that the robot’s IP is **10.66.171.191** and the desktop’s IP is **10.66.171.108** on the same Ethernet network.

For cloning this repository, use:
```bash
git clone https://github.com/aoloo-r/ros1-melodic-humble-bridge.git
```

---

## Overview

- **Docker Image:**  
  The image is based on Ubuntu 22.04 and installs ROS2 Humble (desktop) via apt. It force-installs a partial ROS1 environment (using Ubuntu's `ros-desktop-dev` packages) to emulate a Melodic environment. The image builds the `ros1_bridge` (for bridging ROS1 Melodic ↔ ROS2 Humble) using colcon.

- **Bridge Extraction:**  
  After building the image, a precompiled tarball containing the bridge overlay is generated. This overlay is then extracted locally on your desktop and sourced to run the dynamic bridge.

- **Usage:**  
  The desktop runs the dynamic bridge node, which connects to the ROS1 master on the robot over the network. ROS1 topics (e.g., `/rosout` and `/chatter` from a test talker) are then bridged into the ROS2 environment.

---

## Prerequisites

- **Desktop:**  
  - Ubuntu 22.04 with ROS2 Humble installed  
  - Docker installed

- **Robot:**  
  - Ubuntu 18.04 with ROS1 Melodic installed  
  - Network connectivity between robot and desktop on the same subnet  
  - Example IP configuration:  
    - **Robot IP:** 10.66.171.191  
    - **Desktop IP:** 10.66.171.108

---

## Part 1: Build the Docker Image

1. **Clone the Repository:**

   ```bash
   git clone https://github.com/aoloo-r/ros1-melodic-humble-bridge.git
   cd ros1-melodic-humble-bridge
   ```

2. **Build the Docker Image:**

   ```bash
   docker build -t ros-humble-ros1-bridge-builder .
   ```

   *Note: This build may take a while as it compiles parts of ROS1 from source and builds the bridge.*

---

## Part 2: Extract the Bridge Overlay

After the image is built, run the following command to extract the precompiled bridge overlay tarball:

```bash
docker run --rm ros-humble-ros1-bridge-builder | tar xzvf -
```

This creates a directory named `ros-humble-ros1-bridge` in your current folder.

---

## Part 3: Running the Bridge on the Desktop

### Desktop Setup

1. **Set Environment Variables on the Desktop:**

   Open a terminal on your desktop and run:

   ```bash
   export ROS_MASTER_URI=http://10.66.171.191:11311
   export ROS_IP=10.66.171.108
   ```

2. **Source ROS2 and the Bridge Overlay:**

   ```bash
   source /opt/ros/humble/setup.bash
   cd ros-humble-ros1-bridge
   source install/local_setup.bash
   ```

3. **Launch the Dynamic Bridge:**

   To avoid node name conflicts and force all topics to be bridged, run:

   ```bash
   ros2 run ros1_bridge dynamic_bridge --ros-args -r __node:=dynamic_bridge_desktop -- --bridge-all-topics
   ```

4. **Verify Bridged Topics:**

   Open a new terminal on your desktop (with ROS2 sourced) and run:

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

   You should see topics like `/rosout`, `/parameter_events`, and (if a publisher is active) `/chatter`.

   To view messages on `/chatter`:

   ```bash
   ros2 topic echo /chatter
   ```

---

## Part 4: Setting Up the Robot (ROS1 Melodic)

### Robot Setup

1. **Open a Terminal on the Robot and Set Environment Variables:**

   ```bash
   source /opt/ros/melodic/setup.bash
   export ROS_IP=10.66.171.191
   export ROS_HOSTNAME=10.66.171.191
   export ROS_MASTER_URI=http://10.66.171.191:11311
   ```

2. **Start the ROS Master:**

   ```bash
   roscore
   ```

   Keep this terminal open.

3. **Launch a Test Publisher:**

   In another terminal on the robot, run:

   ```bash
   source /opt/ros/melodic/setup.bash
   rosrun rospy_tutorials talker
   ```

   This will publish messages on the `/chatter` topic.

4. **Verify the Publisher:**

   ```bash
   rostopic info /chatter
   ```

   The publisher’s URI should be listed as something like `http://10.66.171.191:XXXXX/`.  
   *If it still shows a hostname like `segbot`, update your `/etc/hosts` on both the robot and desktop to map `segbot` to `10.66.171.191`.*

---

## Part 5: Troubleshooting

- **Hostname Resolution:**  
  If ROS1 nodes advertise with a hostname (e.g., `segbot`), add an entry in `/etc/hosts` on both the robot and desktop:
  
  ```bash
  sudo nano /etc/hosts
  ```
  
  Add:
  
  ```
  10.66.171.191   segbot
  ```
  
- **Network Connectivity:**  
  Ensure the desktop can ping the robot:
  
  ```bash
  ping 10.66.171.191
  ```
  
- **Bridge Verification:**  
  Use `ros2 topic list` and `ros2 topic echo /chatter` on the desktop to verify that topics from the robot are being bridged.

---

## License

[Include your chosen license here.]

---

## Acknowledgments

This project leverages the official ROS2 Docker images and the `ros1_bridge` package by Open Robotics.
```

Feel free to modify any details (like the license or IP addresses) as needed for your setup.
