void Arduino_setup(void)
{
	string PORTNAMEIN;
    cout << "Input Serial Port (e.g. /dev/ttyUSB0): " << endl;
    cin >> PORTNAMEIN;
	string COMMANDCAST = "rosrun rosserial_python serial_node.py ";
    COMMANDCAST = COMMANDCAST+PORTNAMEIN;
	char *COMMAND = new char [COMMANDCAST.length() + 1];
    std::strcpy(COMMAND, COMMANDCAST.c_str());
    //system(COMMAND);			//Open Arduino port for ROS interface.
	//This line would not work and should be set with a launch file.
	return;
}

