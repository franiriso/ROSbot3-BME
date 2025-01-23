# ROSbot3-BME Project

This project is designed for real-life deployment on a ROSbot3 and includes functionalities such as obstacle avoidance and LiDAR data processing. It also supports simulation environments using Gazebo. Follow this README to set up, run, or contribute to the project.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [System Requirements](#system-requirements)
3. [Setup Instructions](#setup-instructions)
   - [Clone the Repository](#clone-the-repository)
   - [Update the Repository](#update-the-repository)
   - [Build the Workspace](#build-the-workspace)
4. [Running the Project](#running-the-project)
   - [Workflow Overview](#workflow-overview)
5. [Contributing](#contributing)
6. [Appendices](#appendices)
   - [Appendix A: Installing WSL on Windows](#appendix-a-installing-wsl-on-windows)
   - [Appendix B: Installing UTM on macOS](#appendix-b-installing-utm-on-macos)
   - [Appendix C: Installing ROS2 Humble](#appendix-c-installing-ros2-humble)
   - [Appendix D: Installing Gazebo](#appendix-d-installing-gazebo)
   - [Appendix E: Installing Husarion Environment](#appendix-e-installing-husarion-environment)

---

## Project Overview

The ROSbot3-BME project consists of multiple ROS2 nodes implemented in C++ for real-time obstacle avoidance and LiDAR data handling. The directory structure is as follows:

- **include/rosbot_3_bme**: Header files.
- **src/nodes**: Source files for the ROS2 nodes.
- **CMakeLists.txt**: Build configuration for the package.
- **package.xml**: Package metadata.

---

## System Requirements

- **Operating System**: Ubuntu 22.04 (Recommended)
  - If you are using **Windows 10/11**, [install WSL guide](#appendix-a-installing-wsl-on-windows).
  - If you are using **macOS**, [install UTM guide](#appendix-b-installing-utm-on-macos).
- **ROS2 Distribution**: Humble
  - [Install ROS2 Humble guide](#appendix-c-installing-ros2-humble).
- **Simulator**: Gazebo Fortress
  - [Install Gazebo guide](#appendix-d-installing-gazebo).
- **Environment**: Husarion ROSBot
  - [Install Husarion Environment](#appendix-e-installing-husarion-environment).

---

## Setup Instructions

### Clone the Repository

To start using this project, clone the repository:

```bash
git clone https://github.com/your-username/rosbot3-bme.git
cd rosbot3-bme
```

### Update the Repository

Before running or modifying the project, ensure your repository is up to date with the latest changes. Pull the latest updates using:

```bash
git pull origin main
```

### Build the Workspace

Create a workspace directory if not already done:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s <path-to-rosbot3-bme>  # Symlink the project into your workspace
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Running the Project

### Workflow Overview

1. Pull the Latest Changes: Before starting, ensure your local repository on the ROSbot or your computer is up to date:

```bash
git pull origin main
```

2. Test Modifications in Simulation: Test the changes in the Gazebo simulation to ensure they work as intended.

```bash
ROSBOT_SIM
```

Run any example you want to try: 

```bash
ros2 run rosbot_3_bme <test_example>
```

3. Push Changes to GitHub: If the modifications work in the simulation, push them to the repository:

```bash
git add .
git commit -m "Describe the changes"
git push origin main
```

4. Pull Changes on the ROSbot: On the real ROSbot, pull the latest changes from the repository:

```bash
git pull origin main
```

5. Run the Project: Finally, execute the updated project on the ROSbot.

```bash
ros2 run rosbot_3_bme <test_example>
```

---

## Contributing

We welcome contributions to improve the project! You can add some more examples or new subprojects for both the simulation and real implementation of the RosBot3 model.

Here's how you can get started:

1. Fork the repository on GitHub.

2. Create a new branch for your feature or bugfix.

3. Make your changes and test them thoroughly.

4. Push the branch to your fork.

5. Open a pull request on the main repository.

Note: Before submitting, ensure your code follows ROS2 conventions and is properly documented.

---

## Appendices

### Appendix A: Installing WSL on Windows

Open PowerShell as Administrator and run:

```bash
wsl --install Ubuntu-22.04
```

This will install the default version of WSL along with Ubuntu 22.04. Launch Ubuntu and set up your username and password.
For more details, visit the [Microsoft WSL Guide](https://learn.microsoft.com/en-us/windows/wsl/install).

### Appendix B: Installing UTM on macOS

1. Download UTM from here.
2. Create a new virtual machine and install Ubuntu 22.04:
    - Use an Ubuntu 22.04 Server ISO from [Ubuntu Downloads](https://ubuntu.com/download).
    - Follow [UTM’s official guide](https://docs.getutm.app/guides/ubuntu/).
3. Start the virtual machine, and set up Ubuntu.

### Appendix C: Installing ROS2 Humble

Set up the sources list:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
```

Install ROS2 Humble:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
```

Source ROS2:

```bash
source /opt/ros/humble/setup.bash
```

Install colcon:

```bash
sudo apt install -y python3-colcon-common-extensions
```

### Appendix D: Installing Gazebo

First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install -y lsb-release gnupg
```

Install Gazebo Fortress:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

More information can be found in the [Gazebo Simulator Official Guide](https://gazebosim.org/docs/fortress/install_ubuntu/).

Test Gazebo:

```bash
ign gazebo shapes.sdf
```

Note: if it does not work correctly, try running the following line before the test (used for setting the environment variable for GPU usage in the current terminal):

```bash
export LIBGL_ALWAYS_SOFTWARE=0
ign gazebo shapes.sdf
```

If this change works, you can add that line at the end of the setup.bash file to make it persistent:

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=0" >> ~/.bashrc
source ~/.bashrc
```

If issues arise, fall back to CPU rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ign gazebo shapes.sdf
```


### Appendix E: Installing Husarion Environment

1. Create the workspace and clone rosbot-ros repository

```bash
# user@mylaptop:~$
mkdir -p rosbot_ws/src
cd rosbot_ws
git clone https://github.com/husarion/rosbot_xl_ros src/
```

2. Install tools and dependencies.

```bash
# user@mylaptop:~/rosbot_ws$
sudo apt-get update
sudo apt install ros-dev-tools
vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos
vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

3. Build and source your code.

```bash
# user@mylaptop:~/rosbot_ws$
export HUSARION_ROS_BUILD=simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

*Repeat the previous steps until the colcon build has a positive outcome.*

4. Add source command to .bashrc - source the workspace every time a new terminal is opened.

```bash
# user@mylaptop:~/rosbot_ws$
echo 'source ~/rosbot_ws/install/setup.bash' >> ~/.bashrc
```

5. Create alias in .bashrc file to speed up launching a simulator.

```bash
echo "alias ROSBOT_SIM='ros2 launch rosbot_xl_gazebo simulation.launch.py'" >> ~/.bashrc
. ~/.bashrc
```

6. Launch simulation.

```bash
ROSBOT_SIM
```
