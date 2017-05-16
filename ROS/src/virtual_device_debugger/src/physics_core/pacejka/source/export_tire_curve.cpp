#include <iostream>
#include <fstream>
#include <string>

#include "pacejka.h"

void findAndReplace(std::string &source, const std::string find, std::string replace )
{
	size_t j;

	for (;(j = source.find( find )) != std::string::npos;) {
		source.replace(j, find.length(), replace);
	}
}

int main(int argc, char *argv[])
{
	std::cout << "-- H.B Pacejka Magic Formula --" << std::endl << std::endl;

	float load;

	if (argc == 2)
	{
		load = (float)atof(argv[1]);
	}
	else
	{
		std::cout << "Enter load (default 3500N): ";

		std::string input = "";
		std::getline(std::cin, input);

		load = (float)atof(input.c_str());
	}

	if (load > 0)
	{
		std::cout << "! Using load of " << load << " Newtons" << std::endl << std::endl;
	}
	else
	{
		std::cout << "Invalid load entered, using 3500 Newtons" << std::endl;

		load = 3500.0f;
	}

	Pacejka pacejka = Pacejka();

	// Longitudinal coefficients
	pacejka.a0 = 1.3f;
	pacejka.a1 = -49.0f;
	pacejka.a2 = 1216.0f;
	pacejka.a3 = 1632.0f;
	pacejka.a4 = 11.0f;
	pacejka.a5 = 0.006f;
	pacejka.a6 = -0.04f;
	pacejka.a7 = -0.4f;
	pacejka.a8 = 0.003f;
	pacejka.a9 = -0.002f;
	pacejka.a10 = 0.0f;
	pacejka.a111 = -11.0f;
	pacejka.a112 = 0.045f;
	pacejka.a12 = 0.0f;
	pacejka.a13 = 0.0f;
	
	// Lateral coefficients
	pacejka.b0 = 1.57f;
	pacejka.b1 = -48.0f;
	pacejka.b2 = 1338.0f;
	pacejka.b3 = 6.8f;
	pacejka.b4 = 444.0f;
	pacejka.b5 = 0.0f;
	pacejka.b6 = 0.0034f;
	pacejka.b7 = -0.008f;
	pacejka.b8 = 0.66f;
	pacejka.b9 = 0.0f;
	pacejka.b10 = 0.0f;
	
	// Aligning moment coefficients
	pacejka.c0 = 2.46f;
	pacejka.c1 = -2.77f;
	pacejka.c2 = -2.9f;
	pacejka.c3 = 0.0f;
	pacejka.c4 = -3.6f;
	pacejka.c5 = -0.1f;
	pacejka.c6 = 0.0004f;
	pacejka.c7 = 0.22f;
	pacejka.c8 = -2.31f;
	pacejka.c9 = 3.87f;
	pacejka.c10 = 0.0007f;
	pacejka.c11 = -0.05f;
	pacejka.c12 = -0.006f;
	pacejka.c13 = 0.33f;
	pacejka.c14 = -0.04f;
	pacejka.c15 = -0.4f;
	pacejka.c16 = 0.092f;
	pacejka.c17 = 0.0114f;

	pacejka.setCamber(0.0f);
	pacejka.setLoad(load);

	std::ofstream longitudinalOut("longitudinal.csv");
	std::ofstream lateralOut("lateral.csv");
	std::ofstream aligningForceOut("align.csv");

	float longidudinalForce, lateralForce, aligningForce;
	char rowBuffer [50];

	longitudinalOut << "Slip ratio;Longitudinal force" << std::endl;

	std::cout << "> Calculating longitudinal forces.. ";

	for (float slipRatio = -2.0f; slipRatio <= 2.0f; slipRatio += 0.01f)
	{
		pacejka.setSlipRatio(slipRatio);
		pacejka.calculate();

		longidudinalForce = pacejka.getLongitudinalForce();

		sprintf_s(rowBuffer, "%.1f;%.2f", slipRatio, longidudinalForce);

		std::string rowContent(rowBuffer);
		findAndReplace(rowContent, ".", ",");

		longitudinalOut << rowContent << std::endl;
	}

	std::cout << "done!" << std::endl;

	lateralOut << "Slip angle;Lateral force" << std::endl;
	aligningForceOut << "Slip angle;Align force" << std::endl;

	std::cout << "> Calculating lateral and aligning forces.. ";

	for (float slipAngle = -20.0f; slipAngle <= 20.0f; slipAngle += 0.1f)
	{
		pacejka.setSlipAngle(slipAngle);
		pacejka.calculate();

		lateralForce = pacejka.getLateralForce();
		aligningForce = pacejka.getAligningForce();

		sprintf_s(rowBuffer, "%.1f;%.2f", slipAngle, lateralForce);

		std::string rowContent(rowBuffer);
		findAndReplace(rowContent, ".", ",");

		lateralOut << rowContent << std::endl;

		sprintf_s(rowBuffer, "%.1f;%.2f", slipAngle, aligningForce);

		rowContent = std::string(rowBuffer);
		findAndReplace(rowContent, ".", ",");

		aligningForceOut << rowContent << std::endl;
	}

	std::cout << "done!" << std::endl;

	longitudinalOut.close();
	lateralOut.close();
	aligningForceOut.close();
	
	return 0;
}
