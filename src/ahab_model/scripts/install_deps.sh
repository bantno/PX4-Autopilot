#!/usr/bin/env bash
# Install ROS 2 Humble + Gazebo Harmonic for the AHAB sim.
# For the Ubuntu 22.04 (jammy) dev container. Jazzy is 24.04-only, so it does
# not apply here even though the host is 24.04.
set -euo pipefail

# Locale (ROS needs UTF-8; the container has none set)
sudo apt-get update
sudo apt-get install -y locales curl gnupg lsb-release software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 apt source (official ros2-apt-source package, handles the signing key)
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Gazebo (OSRF) apt source
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update

# ROS 2 Humble, Gazebo Harmonic and the Humble<->Harmonic bridge.
# Do NOT also install ros-humble-ros-gz: that one targets Gazebo Fortress and
# conflicts with ros-humble-ros-gzharmonic.
sudo apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  gz-harmonic \
  ros-humble-ros-gzharmonic \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui

# rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# Source ROS in new shells
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
  echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
grep -qxF 'export LANG=en_US.UTF-8' ~/.bashrc || \
  echo 'export LANG=en_US.UTF-8' >> ~/.bashrc

echo "Done. Open a new terminal, then check with: ros2 --help && gz sim --version"
