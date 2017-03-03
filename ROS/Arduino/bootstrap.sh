sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
sudo chmod 777 /usr/share/arduino/libraries/
cp ../../Arduino/FlexiTimer2 /usr/share/arduino/libraries/
cd /usr/share/arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
