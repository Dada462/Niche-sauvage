
sudo apt install -y ros-noetic-mavlink \
ros-noetic-geographic-msgs \
 geographiclib-tools \
 python3-catkin-tools \
 python3-rosinstall-generator \
 python3-osrf-pycommon \
 ros-noetic-joy \
 sshpass

cd $HOME/catkin_ws/src
git clone https://github.com/mavlink/mavros.git
cd mavros
git checkout 0af02a552b7ea981a64aca2bd307ce56020ed4c2
sudo bash mavros/scripts/install_geographiclib_datasets.sh

#https://github.com/mavlink/mavros/issues/963