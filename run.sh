#!/bin/bash
echo "

     A        GGGGGGG     IIIII    LLL        EEEEEEEEE  VVV      VV
    AAA      GG     GG     III     LLL        EEE        VVV      VV
   AA AA    GGG            III     LLL        EEE        VVV      VV
  AA   AA   GGG            III     LLL        EEEEEEEEE   VV     VV 
 AA     AA  GGG  GGGGG     III     LLL        EEE          VV   VV 
AAAAAAAAAAA GGG     GG     III     LLL        EEE           VV VV
AA      AAA  GG     GG     III     LLL        EEE            VVV
AA      AAA   GGGGGGG     IIIII    LLLLLLLLL  EEEEEEEEE       V

"

echo "------------- INITIALIZING STEERING WHEEL -------------"
#initialize Logitech drivers
cd ./LogitechFFDrivers
sudo make load_g29
read -p "Please reset the steering wheel (unplug/plug), then press any key to continue..."

echo "-------------- CUSTOMIZING SYSTEM SETUP ---------------"
cd ../ROS
source ./devel/setup.bash
cd ./src/agile_v_core
#modify old launch files based on input of USB port names if necessary.

#launch tht nodes
echo "------------------ LAUNCHING NODES --------------------"
roslaunch ./AgileVehicle.launch

