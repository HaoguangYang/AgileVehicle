#include "subscribers.h" 

ElectricStat ElectricMon[4];

void readFromWheelsDrv00(const std_msgs::UInt16MultiArray& wheelData)
{
    Enc[0][0].update(wheelData.data[0]);
    Enc[1][0].update_value(wheelData.data[1]);
    return;
}

void readFromWheelsDrv01(const std_msgs::UInt16MultiArray& wheelData)
{
    Enc[0][1].update(wheelData.data[0]);
    Enc[1][1].update_value(wheelData.data[1]);
    return;
}

void readFromWheelsDrv02(const std_msgs::UInt16MultiArray& wheelData)
{
    Enc[0][2].update(wheelData.data[0]);
    Enc[1][2].update_value(wheelData.data[1]);
    return;
}

void readFromWheelsDrv03(const std_msgs::UInt16MultiArray& wheelData)
{
    Enc[0][3].update(wheelData.data[0]);
    Enc[1][3].update_value(wheelData.data[1]);
    return;
}

void readFromWheelsPwr00(const std_msgs::Float32MultiArray& powerData)
{
    ElectricMon[0]._volt = powerData.data[0];
	ElectricMon[0]._ampS = powerData.data[1];
	ElectricMon[0]._ampD = powerData.data[2];
	return;
}

void readFromWheelsPwr01(const std_msgs::Float32MultiArray& powerData)
{
    ElectricMon[1]._volt = powerData.data[0];
	ElectricMon[1]._ampS = powerData.data[1];
	ElectricMon[1]._ampD = powerData.data[2];
	return;
}

void readFromWheelsPwr02(const std_msgs::Float32MultiArray& powerData)
{
    ElectricMon[2]._volt = powerData.data[0];
	ElectricMon[2]._ampS = powerData.data[1];
	ElectricMon[2]._ampD = powerData.data[2];
	return;
}

void readFromWheelsPwr03(const std_msgs::Float32MultiArray& powerData)
{
    ElectricMon[3]._volt = powerData.data[0];
	ElectricMon[3]._ampS = powerData.data[1];
	ElectricMon[3]._ampD = powerData.data[2];
	return;
}

