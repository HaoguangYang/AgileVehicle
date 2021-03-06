#include "physics_core.h"
//#include <thread>
#include <limits>

#include "pacejka.h"
#define R_W 0.315

using namespace boost::numeric::ublas;
//bool no_quit = true;
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

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

	//std::cout << "> Calculating Tire Contact (Friction) Forces.. ";
	
	//See the definitions about the tyre terms.
	float slipRatio = (omega*R_W/std::max(fabs(v_wx), std::numeric_limits<double>::epsilon())*sgn(v_wx) - 1)*100.0;//More precisely, R_W should be R_W-x(wheel_z)
	float slipAngle = -atan(v_wy/v_wx);
	pacejka.setSlipRatio(slipRatio);
    pacejka.setSlipAngle(slipAngle);
	pacejka.calculate();

	F_long = pacejka.getLongitudinalForce();
	F_lat = pacejka.getLateralForce();
	T_ali = pacejka.getAligningForce();

    printf("F_long= %f    ; F_lat= %f  (N)    ; T_ali= %f  (Nm)\n", F_long, F_lat, T_ali);
	//std::cout << "done!" << std::endl;
	
	return 0;
}

//template<class T>
bool InvertMatrix (const matrix<double>& input, matrix<double>& inverse) {
 	//using namespace boost::numeric::ublas;
 	typedef permutation_matrix<std::size_t> pmatrix;
 	// create a working copy of the input
 	matrix<double> A(input);
 	// create a permutation matrix for the LU-factorization
 	pmatrix pm(A.size1());

 	// perform LU-factorization
 	int res = lu_factorize(A,pm);
        if( res != 0 ) return false;

 	// create identity matrix of "inverse"
 	inverse.assign(identity_matrix<double>(A.size1()));

 	// backsubstitute to get the inverse
 	lu_substitute(A, pm, inverse);

 	return true;
 }


#define nDOF 24
vector<double> d1(nDOF);
vector<double> d2(nDOF);
vector<double> d3(nDOF);
vector<double> v2(nDOF);
vector<double> a2(nDOF);
symmetric_matrix<double> K(nDOF, nDOF);
symmetric_matrix<double> C(nDOF, nDOF);
diagonal_matrix<double> M(nDOF);
vector<double> Q(nDOF);
vector<double> d0(nDOF);
vector<double> v0(nDOF);
vector<double> a0(nDOF);
float sim_time = 0.0;
const double dt = 0.025;
//double ctrlVolt[4];
//double AngSpeed[4];

void compute_main(matrix<double>& invM, const double c0, const double c1, const double c2, vector<double>& d1, vector<double>& d2){
	vector<double> f_eff(nDOF);
	f_eff = Q - prod((K-c2 * M), d2) - prod((c0 * M-c1 * C), d1);
	d3 = prod(invM, f_eff);
	a2 = c0 * (d1-2.0*d2+d3);
	v2 = c1 * (d3-d1);
	d1 = d2;
	d2 = d3;
}

void assem_M_matrix(diagonal_matrix<double> Mass){
	const double M_t = 30.0;
	const double M_v = 400.0;
	//Temporary Values Below!
	const double I_vz = 1.0;//Yaw/Heading
	const double I_vx = 1.0;//Roll
	const double I_vy = 1.0;//Pitch
	const double I_tz = 1.0;//Wheel Alignment
	const double I_ty = 1.0;//Wheel Roll
	double m_vec[nDOF] = {M_t, M_t, M_t, I_tz, I_ty, \
			              M_t, M_t, M_t, I_tz, I_ty, \
			              M_t, M_t, M_t, I_tz, I_ty, \
			              M_t, M_t, M_t, I_tz, I_ty, \
			              M_v, M_v, M_v, I_vz};
	for (int i=0; i<nDOF; i++){
	    Mass(i,i) = m_vec[i];
	}
}

void BLDC_model(double ctrlVolt, double AngSpeed, double Torque)
{
	const double gain1 = 0.09549296586;
	const double gain2 = 10.33;		//N.m/V
	const double gain3 = 0.6;		//Friction-induced Torque
    if (ctrlVolt >=0)
        Torque = std::max((ctrlVolt-1.2)*gain2-gain1*AngSpeed-gain3, 0.0);   //ctrlVolt in (0,5)
    else
        Torque = std::min((ctrlVolt-1.2)*gain2-gain1*AngSpeed-gain3, 0.0);
}


void update_param(matrix<double>& invM, const double &tc0, const double &tc1){
	typedef boost::array<matrix<double>, 4> wheel_part_matrices;
	typedef boost::array<vector<double>, 4> wheel_part_vectors;
    wheel_part_matrices K1;
    wheel_part_matrices d1;
	wheel_part_vectors v_w;
	K1.assign(symmetric_matrix<double>(3,3));
	d1.assign(symmetric_matrix<double>(3,3));
	v_w.assign(vector<double>(2));
	double Qtmp1, Qtmp2, Qtmp3, Qtmp4; //Use as needed
	//identity_matrix<double> I(3);
	//matrix<double> K_2(6,6);
	//matrix<double> K_3(6,6);
	//matrix<double> K_4(6,6);
	//Constriaint Matries on displacements, velocity, and acceleration
	//symmetric_matrix<double> constrA(nDOF, nDOF);
	//symmetric_matrix<double> constrV(nDOF, nDOF);
	//symmetric_matrix<double> constrX(nDOF, nDOF);
	const double k_t = 2.65e+05;
	const double d_t = 500.0;
	const double g = -9.81;
	const double d_St = 100.0;
	
	double k_S, k_C, k_R1, k_R2, d_S, d_C, d_R1, d_R2;
	double alpha_1[4] , alpha_2[4];
	
	//Submatrices
	for (int i = 0; i < 4; i++){
		double C1 = cos(alpha_1[i]);
		double S1 = sin(alpha_1[i]);
		double C2 = cos(alpha_2[i]);
		double S2 = sin(alpha_2[i]);
		K1[i](0,0) = 2*k_S*C1*C1+k_R2;
		K1[i](0,1) = 0.0;
		K1[i](0,2) = 0.0;
		K1[i](1,1) = k_C*C2*C2+2*k_R1;
		K1[i](1,2) = k_C*C2*S2;
		K1[i](2,2) = k_C*S2*S2+2*k_S*S1*S1;
		d1[i](0,0) = 2*d_S*C1*C1+d_R2;
		d1[i](0,1) = 0.0;
		d1[i](0,2) = 0.0;
		d1[i](1,1) = d_C*C2*C2+2*d_R1;
		d1[i](1,2) = d_C*C2*S2;
		d1[i](2,2) = d_C*S2*S2+2*d_S*S1*S1;
	}
	
	//Assembly: Dims = [wheel0_x wheel0_y wheel0_z wheel0_theta wheel0_roll		\
					 wheel1_x ... 										\
					 ... \
					 ... 			wheel3_z wheel3_theta wheel3_roll		\
					 chassis_x chassis_y chassis_z chassis_hdg]
	const int scatter[4][6] =  {0,	1,	2,	20,	21,	22, \
								5,	6,	7,	20,	21,	22, \
								10,	11,	12,	20,	21,	22, \
								15,	16,	17,	20,	21,	22};//Suspensions
	const int scatter_tyre[4] = {2, 7, 12, 17};
	const int scatter_steer[4] = {3, 8, 13, 18};
	const int scatter_wheel_movement[4][2] = {0, 1, 5, 6, 10, 11, 15, 16};
	const int scatter_mass[5] = {2, 7, 12, 17, 22};
	const int scatter_ang_inert[5] = {3, 8, 13, 18, 23}; //Rolling inertia of each wheel and the inertia of vehicle heading.
	
	//Re-init the matrices
	for (int i = 0; i<nDOF; i++){
	    for (int j = 0; j<nDOF; j++){
			K(i,j) = 0.0;
			C(i,j) = 0.0;
	    }
	    Q(i) = 0.0;
	}
	for (int i=0; i<4; i++){
		for (int j=0; j<3; j++)
		for (int k=0; k<3; k++){
		    //Suspensions
			K(scatter[i][j], scatter[i][k]) += K1[i](j,k);
			K(scatter[i][j+3], scatter[i][k+3]) += K1[i](j,k);
			K(scatter[i][j], scatter[i][k+3]) += -K1[i](j,k);
			K(scatter[i][j+3], scatter[i][k]) += -K1[i](j,k);
			C(scatter[i][j], scatter[i][k]) += d1[i](j,k);
			C(scatter[i][j+3], scatter[i][k+3]) += d1[i](j,k);
			C(scatter[i][j], scatter[i][k+3]) += -d1[i](j,k);
			C(scatter[i][j+3], scatter[i][k]) += -d1[i](j,k);
		}
		//Tyres
		K(scatter_tyre[i], scatter_tyre[i]) += k_t;
		C(scatter_tyre[i], scatter_tyre[i]) += d_t;
		//Steering Pod Friction
		C(scatter_steer[i], scatter_steer[i]) += d_St;
		C(scatter_steer[i], 23) -= d_St;
		C(23, scatter_steer[i]) -= d_St;
		C(23, 23) += d_St;
		//BLDC Driving Forces
		
		BLDC_model(ctrlVolt[i], AngSpeed[i], Qtmp1);
		Q(scatter_tyre[i]) += Qtmp1;
		//Tire Loads
		float load = k_t * d3(scatter_tyre[i]) + d_t * v2(scatter_tyre[i]) + \
					 M(scatter_tyre[i], scatter_tyre[i]) * a2(scatter_tyre[i]);
		//Transform global wheel movement to wheel axis systems for the calculation of slip ratio and therefore tyre forces. Alignment Moments are also considered.
		double v_wi_glob[2];
		double S = sin(d3(scatter_steer[i]));
		double C = cos(d3(scatter_steer[i]));
		v_wi_glob[0] = v2(scatter_wheel_movement[i][0]);
		v_wi_glob[1] = v2(scatter_wheel_movement[i][1]);
		v_w[i](0) = v_wi_glob[0]*C+v_wi_glob[1]*S;
		v_w[i](1) = -v_wi_glob[0]*S+v_wi_glob[1]*C;
		getTireForces(load, AngSpeed[i], v_w[i](0), v_w[i](1), Qtmp1, Qtmp2, Qtmp3);
		if (Qtmp1 == NAN) Qtmp1 = 0.;
		if (Qtmp2 == NAN) Qtmp2 = 0.;
		if (Qtmp3 == NAN) Qtmp3 = 0.;
		Q(scatter_wheel_movement[i][1]) += Qtmp1;
		Q(scatter_wheel_movement[i][0]) += Qtmp2;
		Q(scatter_steer[i]) += Qtmp3;
		
		//Inertial Forces added to Q
		Qtmp1 = M(scatter[i][0], scatter[i][0])*a2(scatter[i][3]);
		Qtmp2 = M(scatter[i][1], scatter[i][1])*a2(scatter[i][4]);
		Qtmp3 = M(scatter[i][2], scatter[i][2])*a2(scatter[i][5]);
		Qtmp4 = M(scatter_ang_inert[i], scatter_ang_inert[i]) * a2(scatter_ang_inert[i]);
		Q(scatter[i][0]) += Qtmp1;
		Q(scatter[i][1]) += Qtmp2;
		Q(scatter[i][2]) += Qtmp3;
		Q(scatter_ang_inert[i]) += Qtmp4; 
		Q(scatter[i][3]) -= Qtmp1;
		Q(scatter[i][4]) -= Qtmp2;
		Q(scatter[i][5]) -= Qtmp3;
		Q(scatter_ang_inert[4]) -= Qtmp4;
		//Damping Forces added to Q
		
	}
	
	symmetric_matrix<double> M_eff(nDOF, nDOF);
	//Update Effective Mass Matrix.
	M_eff = tc0*M + tc1*C;
	int err = InvertMatrix(M_eff, invM);
	for (int i = 0; i<5; i++){
		Q(scatter_mass[i]) -= M(scatter_mass[i], scatter_mass[i]) * g;	//Gravity
	}
}

//Solution of dynamic model using central difference with explicit integration
int dyna_core(void){
	//vector<double> d0(nDOF), v0(nDOF), a0(nDOF), d1(nDOF);
	//Integration constants
	
	const double c0 = 1/dt/dt;
	const double c1 = 0.5/dt;
	const double c2 = 2*c0;
	//Displacement at -dt
	vector<double> d1(nDOF);
	d1 = d0 - dt * v0 + a0/c2;
	vector<double> d2(nDOF);
	d2 = d0;
	//Form Equivilant Mass Matrix
	//diagonal_matrix<double> M_mat(nDOF, M.data());
	matrix<double> invM(nDOF,nDOF);
	assem_M_matrix(M);
	//Constriaint Matries on displacements, velocity, and acceleration
	//Compute the motion and update the parameters.
	compute_main (invM, c0, c1, c2, d1, d2);
	while (no_quit){
	    
	    update_param (invM, c0, c1);
	    
    	compute_main (invM, c0, c1, c2, d1, d2);
    	
	    sim_time += dt;
	    printf("Time: %f \n Disp:\t  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f\nVelo:\t  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f  %f\n\
	  %f  %f  %f  %f\n" , sim_time, d3, v2);
	  
	    usleep (25000);
	}
	return 0;
}
