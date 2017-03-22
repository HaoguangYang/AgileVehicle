#!/bin/bash
echo "
     A        GGGGGGG     IIIII    LLL        EEEEEEEEE  VV       VV
    AAA      GG     GG     III     LLL        EEE        VV       VV
   AA AA    GGG            III     LLL        EEE        VV       VV
  AA   AA   GGG            III     LLL        EEEEEEEEE   VV     VV 
 AA     AA  GGG  GGGGG     III     LLL        EEE          VV   VV 
AAAAAAAAAAA GGG     GG     III     LLL        EEE           VV VV
AA       AA  GG     GG     III     LLL        EEE            VVV
AA       AA   GGGGGGG     IIIII    LLLLLLLLL  EEEEEEEEE       V
"

echo "------------ MAKING STEERING WHEEL DRIVERS ------------"
cd ./LogitechFFDrivers
make

echo "----------------- MAKING ROS PACKAGES -----------------"
cd ../ROS
catkin_make

echo "-------------- CUSTOMIZING SYSTEM SETUP ---------------"
cd ./src/agile_v_core
#modifying old launch files based on input of USB port names.
configFile="./AgileVehicle.launch"
oldpattern="<param name='~port' value='"
sed -i.old "/${oldpattern}/d" $configFile

read -p "Input Serial Port for Wheel-00 (e.g. /dev/ttyUSB0): " -e port0
match='<node pkg="rosserial_python" type="serial_node.py" name="wheel00" output="screen">'
insert="<param name='~port' value='$port1' />"
sed -i "/${match}/a${insert}" $configFile

read -p "Input Serial Port for Wheel-01 (e.g. /dev/ttyUSB0): " -e port1
match='<node pkg="rosserial_python" type="serial_node.py" name="wheel01" output="screen">'
insert="<param name='~port' value='$port1' />"
sed -i "/${match}/a${insert}" $configFile

read -p "Input Serial Port for Wheel-02 (e.g. /dev/ttyUSB0): " -e port2
match='<node pkg="rosserial_python" type="serial_node.py" name="wheel02" output="screen">'
insert="<param name='~port' value='$port2' />"
sed -i "/${match}/a${insert}" $configFile

read -p "Input Serial Port for Wheel-03 (e.g. /dev/ttyUSB0): " -e port3
match='<node pkg="rosserial_python" type="serial_node.py" name="wheel03" output="screen">'
insert="<param name='~port' value='$port3' />"
sed -i "/${match}/a${insert}" $configFile

echo "----------------- MAKING ARDUINO CODE -----------------"
cd ../../Arduino

sed 's/01"/00"/g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port0 Arduino.ino
sed 's/00"/01"/g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port1 Arduino.ino
sed 's/01"/02"/g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port2 Arduino.ino
sed 's/02"/03"/g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port3 Arduino.ino
sed 's/03"/01"/g' Arduino.ino

echo "------------------- COMPILATION DONE ------------------"

