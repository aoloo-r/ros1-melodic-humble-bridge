
## Overview

- **Docker Image:**  
  The image is based on Ubuntu 22.04 and installs ROS2 Humble (desktop) via apt. It force-installs a partial ROS1 environment (using Ubuntu's `ros-desktop-dev` packages) to emulate a Melodic environment. The image then builds the `ros1_bridge` (bridging ROS1 Melodic ↔ ROS2 Humble) using colcon.

- **Bridge Extraction:**  
  After building the image, a precompiled tarball containing the bridge overlay is generated. This overlay is extracted locally on your desktop and sourced to run the dynamic bridge.

- **Usage:**  
  The dynamic bridge node runs on your desktop and connects to a ROS1 master (to be configured externally) so that ROS1 topics are bridged into the ROS2 environment.

---

## Prerequisites

- **Desktop:**  
  - Ubuntu 22.04 with ROS2 Humble installed  
  - Docker installed

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

   *Note: The build process may take a while as it compiles parts of ROS1 from source and builds the bridge.*

---

## Part 2: Extract the Bridge Overlay

After the image is built, extract the precompiled bridge overlay tarball by running:

```bash
docker run --rm ros-humble-ros1-bridge-builder | tar xzvf -
```

This will create a directory named `ros-humble-ros1-bridge` in your current folder.

---

## Part 3: Running the Bridge on Your Desktop

### Desktop Setup

1. **Set Environment Variables:**

   Open a terminal on your desktop and set the following environment variables (adjust the IP addresses as needed):

   ```bash
   export ROS_MASTER_URI=http://<ROS1_MASTER_IP>:11311
   export ROS_IP=<YOUR_DESKTOP_IP>
   ```

   For example, if your desktop IP is `10.66.171.108` and the ROS1 master is reachable at `10.66.171.191`, then:

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

   To avoid node name conflicts and force bridging of all topics, run:

   ```bash
   ros2 run ros1_bridge dynamic_bridge --ros-args -r __node:=dynamic_bridge_desktop -- --bridge-all-topics
   ```

4. **Verify Bridged Topics:**

   Open a new terminal on your desktop (with ROS2 sourced) and run:

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

   You should see topics like `/rosout`, `/parameter_events`, etc. To view messages on a topic (e.g., `/chatter`), run:

   ```bash
   ros2 topic echo /chatter
   ```

---

