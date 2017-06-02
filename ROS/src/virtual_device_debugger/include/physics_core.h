#ifndef PHYSICS_CORE_H
#define PHYSICS_CORE_H

#include <iostream>
#include <fstream>
#include <string>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/symmetric.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/array.hpp>

using namespace boost::numeric::ublas;

bool getTireForces(float load, float omega, float v_wx,
                  float v_wy, float F_lat, float F_long, float T_ali);

bool InvertMatrix (const matrix<double>& input, matrix<double>& inverse);

void compute_main(matrix<double>& invM, const double c0, const double c1, const double c2, vector<double>& d1, vector<double>& d2,\
				  vector<double>& d3, vector<double>& v2, vector<double>& a2);

void assem_M_matrix(diagonal_matrix<double> Mass);

void BLDC_model(double ctrlVolt, double AngSpeed, double Torque);

void update_param(matrix<double>& invM, const double &tc0, const double &tc1, const double *ctrlVolt, const double *AngSpeed,\
                  const vector<double> d3, const vector<double> v2, const vector<double> a2);

int dyna_core(double *ctrlV, double *angV);

#endif
