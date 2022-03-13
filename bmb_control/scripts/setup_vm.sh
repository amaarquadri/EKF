# Update system
sudo apt update
sudo apt upgrade

# Install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Install other tools
sudo apt install git
sudo apt install liburdfdom-tools
sudo apt install python3-catkin-tools

# Create workspace
cd ~
mkdir catkin_ws
cd catkin_ws
catkin init
mkdir src
cd src
git clone https://www.github.com/amaarquadri/fydp.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build

# Update .bashrc file
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Update system
sudo apt update
sudo apt upgrade

