echo "This file is written by adnanhd..."
sleep 1
echo "Waiting for respecting before him..."
sleep 3
echo "just kidding..."
sleep 1
sudo apt-get install -y ros-melodic-ros-control
sudo apt-get install -y ros-melodic-gazebo-ros-control
sudo apt-get install -y ros-melodic-unique-identifier
sudo apt-get install -y ros-melodic-geographic-info
sudo apt-get install -y ros-melodic-laser-geometry
sudo apt-get install -y ros-melodic-tf-conversions
sudo apt-get install -y ros-melodic-tf2-geometry-msgs
sudo apt-get install -y ros-melodic-joy

sed -i -e 's/option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" ON)/option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" OFF)/g' hector_quadrotor/hector_quadrotor_gazebo/urdf/CMakeLists.txt

sed -i -e 's/option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" ON)/option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" OFF)/g' hector_quadrotor/hector_quadrotor_gazebo/urdf/CMakeLists.txt

# this is to deactivate warnings
sed -i -e 's/add_dependencies(landing_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt

sed -i -e 's/add_dependencies(pose_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt

sed -i -e 's/add_dependencies(takeoff_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt

sed -i -e 's/add_dependencies(hector_quadrotor_controllers hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor_controllers/CMakeLists.txt
