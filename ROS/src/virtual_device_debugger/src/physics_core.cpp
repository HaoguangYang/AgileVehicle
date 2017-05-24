#include <iostream>
#include <fstream>
#include <string>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include "pacejka.h"
#define R_W 0.315

using namespace boost::numeric::ublas;

bool getTireForces(float load, float omega, float v_wx,
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
        return 1;
	}

	Pacejka pacejka = Pacejka();

	// Longitudinal coefficients
	pacejka.a0=1.5;
    pacejka.a1=-40;
    pacejka.a2=1600;
    pacejka.a3=2600;
    pacejka.a4=8.7;
    pacejka.a5=0.014;
    pacejka.a6=-0.24;
    pacejka.a7=1.0;
    pacejka.a8=-0.03;
    pacejka.a9=-0.0013;
    pacejka.a10=-0.15;
    pacejka.a111=-8.5;
    pacejka.a112=-0.29;
    pacejka.a12=17.8;
    pacejka.a13=-2.4;
    
    //Lateral coefficients
    pacejka.b0=1.5;
    pacejka.b1=-80;
    pacejka.b2=1950;
    pacejka.b3=23.3;
    pacejka.b4=390;
    pacejka.b5=0.05;
    pacejka.b6=0;
    pacejka.b7=0.055;
    pacejka.b8=-0.024;
    pacejka.b9=0.014;
    pacejka.b10=0.26;
    
    //Aligning moment coefficients
    pacejka.c0=2.2;
    pacejka.c1=-3.9;
    pacejka.c2=-3.9;
    pacejka.c3=-1.26;
    pacejka.c4=-8.2;
    pacejka.c5=0.025;
    pacejka.c6=0;
    pacejka.c7=0.044;
    pacejka.c8=-0.58;
    pacejka.c9=0.18;
    pacejka.c10=0.043;
    pacejka.c11=0.048;
    pacejka.c12=-0.0035;
    pacejka.c13=-0.18;
    pacejka.c14=0.14;
    pacejka.c15=-1.029;
    pacejka.c16=0.27;
    pacejka.c17=-1.1;
	
	
	pacejka.setCamber(0.0f);
	pacejka.setLoad(load);

	std::cout << "> Calculating Tire Contact (Friction) Forces.. ";

	float slipRatio = 0;//getSlipRatio(omega, v_wx);
	float slipAngle = 0;//getSlipAngle(v_wx, v_wy);
	pacejka.setSlipRatio(slipRatio);
    pacejka.setSlipAngle(slipAngle);
	pacejka.calculate();

	F_long = pacejka.getLongitudinalForce();
	F_lat = pacejka.getLateralForce();
	T_ali = pacejka.getAligningForce();

    printf("F_long= %f    ; F_lat= %f  (N)    ; T_ali= %f  (Nm)", F_long, F_lat, T_ali);
	std::cout << "done!" << std::endl;
	
	return 0;
}

template<class T>
bool InvertMatrix? (const ublas::matrix<T>& input, ublas::matrix<T>& inverse) {
 	using namespace boost::numeric::ublas;
 	typedef permutation_matrix<std::size_t> pmatrix;
 	// create a working copy of the input
 	matrix<T> A(input);
 	// create a permutation matrix for the LU-factorization
 	pmatrix pm(A.size1());

 	// perform LU-factorization
 	int res = lu_factorize(A,pm);
        if( res != 0 ) return false;

 	// create identity matrix of "inverse"
 	inverse.assign(ublas::identity_matrix<T>(A.size1()));

 	// backsubstitute to get the inverse
 	lu_substitute(A, pm, inverse);

 	return true;
 }

void compute_main(matrix<T>& K, matrix<T>& M, matrix<T>& C, matrix<T>& Q, matrix<T>& invM, \
				  const double c0, const double c1, const double c2, vector<T>& d0, vector<T>& d1, vector<T>& d2){
	
}

void update_param(matrix<T>& K, matrix<T>& C, matrix<T>& Q, matrix<T>& invM){
	
}

//Solution of dynamic model using central difference with explicit integration
int dyna_core(matrix<T>& K, matrix<T>& M, matrix<T>& C, vector<T>& Q, \
			   const vector<T>& d0, const vector<T>& v0, const vector<T>& a0, const double dt){
	int nDOF=23;
	//vector<double> d0(nDOF), v0(nDOF), a0(nDOF), d1(nDOF);
	//Integration constants
	const double c0 = 1/dt/dt;
	const double c1 = 0.5/dt;
	const double c2 = 2*c0;
	const double c3 = 1/c2;
	//Displacement at -dt
	vector<double> d1(nDOF) = d0 - dt*v0 + c3*a0;
	vector<double> d2(nDOF) = d0;
	//Form Equivilant Mass Matrix
	//diagonal_matrix<double> M_mat(nDOF, M.data());
	matrix<double> M_eff(nDOF, nDOF) = c0*M + c1*C;
	matrix<double> invM(nDOF,nDOF);
	int err = InvertMatrix(M_eff, invM);
	
	//Compute the motion and update the parameters.
	std::thread loop = std::thread(compute_main, K, M, C, Q, invM, c0, c1, c2, d0, d1, d2);
	std::thread update = std::thread(update_param, K, C, Q, invM);
	loop.join();
	update.join();
	return EXIT_SUCCESS;
}