sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
sudo chmod 777 /usr/share/arduino/libraries/
cp -R ../../Arduino/FlexiTimer2 /usr/share/arduino/libraries/
cd /usr/share/arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .

match="#elif defined(__AVR_ATmega328P__)"
oldpattern="typedef NodeHandle_<ArduinoHardware, 8, 8, 200, 200> NodeHandle;"
insert="typedef NodeHandle_<ArduinoHardware, 8, 8, 200, 200> NodeHandle;"
sed -i "/${oldpattern}/d" ./ros_lib/ros.h
sed -i "/${match}/a${insert}" ./ros_lib/ros.h

