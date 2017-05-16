#include <iostream>
#include <fstream>
#include <string>

#include "pacejka.h"
#define R_W 0.315

int getTireForces(float load, float omega, float v_wx,
                  float v_wy, float F_lat, float F_long, float T_ali)
{
	if (load >= 0)
	{
		std::cout << "Load of wheel is " << load << " N" << std::endl << std::endl;
	}
	else
	{
		std::cout << "Wheel is in the air!" << std::endl;
        F_lat = 0.0;
        F_long = 0.0;
        T_ali = 0.0;
        return;
	}

	Pacejka pacejka = Pacejka();

	// Longitudinal coefficients
	pacejka.a0=1.5
    pacejka.a1=-40
    pacejka.a2=1600
    pacejka.a3=2600
    pacejka.a4=8.7
    pacejka.a5=0.014
    pacejka.a6=-0.24
    pacejka.a7=1.0
    pacejka.a8=-0.03
    pacejka.a9=-0.0013
    pacejka.a10=-0.15
    pacejka.a111=-8.5
    pacejka.a112=-0.29
    pacejka.a12=17.8
    pacejka.a13=-2.4
    
    //Lateral coefficients
    pacejka.b0=1.5
    pacejka.b1=-80
    pacejka.b2=1950
    pacejka.b3=23.3
    pacejka.b4=390
    pacejka.b5=0.05
    pacejka.b6=0
    pacejka.b7=0.055
    pacejka.b8=-0.024
    pacejka.b9=0.014
    pacejka.b10=0.26
    
    //Aligning moment coefficients
    pacejka.c0=2.2
    pacejka.c1=-3.9
    pacejka.c2=-3.9
    pacejka.c3=-1.26
    pacejka.c4=-8.2
    pacejka.c5=0.025
    pacejka.c6=0
    pacejka.c7=0.044
    pacejka.c8=-0.58
    pacejka.c9=0.18
    pacejka.c10=0.043
    pacejka.c11=0.048
    pacejka.c12=-0.0035
    pacejka.c13=-0.18
    pacejka.c14=0.14
    pacejka.c15=-1.029
    pacejka.c16=0.27
    pacejka.c17=-1.1
	
	
	pacejka.setCamber(0.0f);
	pacejka.setLoad(load);

	std::cout << "> Calculating Tire Contact (Friction) Forces.. ";

	float slipRatio = getSlipRatio(omega, v_wx);
	float slipAngle = getSlipAngle(v_wx, v_wy);
	pacejka.setSlipRatio(slipRatio);
    pacejka.setSlipAngle(slipAngle);
	pacejka.calculate();

	F_long = pacejka.getLongitudinalForce();
	F_lat = pacejka.getF_lat();
	T_ali = pacejka.getT_ali();

    printf("F_long= %f    ; F_lat= %f  (N)    ; T_ali= %f  (Nm)", F_long, F_lat, T_ali);
	std::cout << "done!" << std::endl;
	
	return 0;
}
