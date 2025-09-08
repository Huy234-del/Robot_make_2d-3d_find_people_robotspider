# Cập nhật hệ thống
sudo apt update && sudo apt upgrade -y

# Cài đặt ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Cài đặt OpenCV và các thư viện liên quan
sudo apt install python3-opencv libopencv-dev libtesseract-dev

# Cài đặt thư viện cho cảm biến
sudo apt install i2c-tools libi2c-dev
pip3 install adafruit-circuitpython-vl53l1x smbus2

# Cài đặt PCL
sudo apt install libpcl-dev ros-humble-pcl-ros

# Cài đặt Coral Edge TPU
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt update
sudo apt install libedgetpu1-std python3-pycoral

# Tạo workspace
mkdir -p ~/rescue_robot_ws/src
cd ~/rescue_robot_ws/src
sudo python3 setup.py install