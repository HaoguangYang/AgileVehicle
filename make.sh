#!/bin/bash
echo "
                             ___,,,,,,___                             
                      ._aaw@@@@@@@@@@@@@@@@wws,.                      
                  ._wg@@@@@@@@@@@@@@@@@@@@@@@@@@gw,.                  
               .aw@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@gc.               
             _w@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@w,             
           _m@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@g,           
         _y@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@g,         
        j@@@@@@@@@@@@@@@@@P   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@a        
      _y@@@@@@@@@@@@@@@@@P    W@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@g.      
     _m@@@@@@@@@@@@@@@@@F     l@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@m,     
    _@@@@@@@@@@@@@@@@@@f      \@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@m,    
   .m@@@@@@@@@@@@@@@@W'        @@@@^^^^^@@@@@@@@@@@@@@@P^^^^^@@@@m    
   j@@@@@@@@@@@@@@@@@'   .m    |@@@     @@@@@@@@@@@@@@P     /@@@@@L   
  _@@@@@@@@@@@@@@@@@_____m@;   \@@@\    \@@@@@@@@@@@@F     /@@@@@@@.  
  j@@@@@@@@@@@@@@@@*****@@@@    @@@|    .@@@@@@@@@@@F     /@@@@@@@@[  
  m@@@@@@@@@@@@@@@@@@@@/@@@k    @@@@     W@@@@@@@@@P     /@@@@@@@@@k  
 .@@@@@@@@@@@@@@@____/W@@@@@    \@@@     W@@@@@@@@______/@@@@@@@@@@W  
 .@@@@@@@@@@@@@*****@@@@@@@@.    @@@     V@@@@@@@******@@@@@@@@@@@@W  
  @@@@@@@@@@@W*    sWW@@@@@@\    @@@\    i@@@@@@@@@@@@@@@@@@@@@@@@@#  
  @@@@@@@@@@P     jW@@@@@@@@m    \@@j    ;@@@@W______@@@@@@@@@@@@@@E  
  ]@@@@@@@@f     j@@@@@@@@@@@i    V@@L    @@@@*****@@@@@@@@@@@@@@@@[  
  *@@@@@@@/    .y@@@@@@@@@@@@k    l@@m    l@P    /m@@@@@@@@@@@@@@@@*  
   ]@@@@@'    .m@@@@@@@@@@@@@m    \@@@    iW    /@@@@@@@@@@@@@@@@@[   
    @@@@______@@@@@@@@@@@@@@@@_____@@@k   *    /@@@@@@@@@@@@@@@@@E    
    *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       j@@@@@@@@@@@@@@@@@@*    
     *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.     j@@@@@@@@@@@@@@@@@@*     
      *4@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@k   .m@@@@@@@@@@@@@@@@@P       
        ?@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@___m@@@@@@@@@@@@@@@@W!        
         -9@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@P'         
           *9@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@P'           
             -9@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@?'             
                *9W@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T*                
                   *?9@@@@@@@@@@@@@@@@@@@@@@@@@@V?^                   
                       '*?9W@@@@@@@@@@@@@@WT?*'                       
                              **********                              
               ___   _____   _   _       _____   _     _  
              /   | /  ___| | | | |     | ____| | |   / / 
             / /| | | |     | | | |     | |__   | |  / /  
            / / | | | |  _  | | | |     |  __|  | | / /   
           / /  | | | |_| | | | | |___  | |___  | |/ /    
          /_/   |_| \_____/ |_| |_____| |_____| |___/     

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
insert="<param name='~port' value='$port0' />"
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
echo "#### Arduino 1.5.0 or higher version is REQUIRED. ####"


sed -i 's/0x"/00"/g' Arduino.ino
sed -i 's/_zero=0; /_zero=231; /g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port0 $(pwd)/Arduino.ino
sed -i 's/00"/01"/g' Arduino.ino
sed -i 's/_zero=231; /_zero=2646; /g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port1 $(pwd)/Arduino.ino
sed -i 's/01"/02"/g' Arduino.ino
#sed -i 's/ctrl_var.data[1]*throttle/ctrl_var.data[1]*throttle+15/g' Arduino.ino
sed -i 's/_zero=2646; /_zero=1895; /g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port2 $(pwd)/Arduino.ino
sed -i 's/02"/03"/g' Arduino.ino
#sed -i 's/ctrl_var.data[1]*throttle+15/ctrl_var.data[1]*throttle-10/g' Arduino.ino
sed -i 's/_zero=1895; /_zero=2297; /g' Arduino.ino
arduino --upload --board arduino:avr:nano:cpu=atmega328 --port $port3 $(pwd)/Arduino.ino
sed -i 's/03"/0x"/g' Arduino.ino
#sed -i 's/ctrl_var.data[1]*throttle-10/ctrl_var.data[1]*throttle/g' Arduino.ino
sed -i 's/_zero=2297; /_zero=0; /g' Arduino.ino

echo "------------------- COMPILATION DONE ------------------"

