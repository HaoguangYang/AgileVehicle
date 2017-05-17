#include "pacejka.h"

#include <math.h>

Pacejka::Pacejka()
{
	a0 = a1 = a2 = a3 = a4 = a5 = a6 = a7 = a8 = a9 = a10 = a111 = a112 = a12 = a13 = 0;
	b0 = b1 = b2 = b3 = b4 = b5 = b6 = b7 = b8 = b9 = b10 = 0;
	c0 = c1 = c2 = c3 = c4 = c5 = c6 = c7 = c8 = c9 = c10 = c11 = c12 = c13 = c14 = c15 = c16 = c17 = 0;

	camber = 0.0f;
	slipAngle = 0.0f;
	slipPercentage = 0.0f;

	longitudinalForce = lateralForce = load = aligningForce = 0.0f;
	longitudinalStiffness = 0.0f;
	latStiffness = 0.0f;
	maxLongitudinalForce = 0.0f;
	maxLateralForce = 0.0f;
}

void Pacejka::calculate()
{
	// Calculate long. force (and long. stiffness plus max long. force)
	longitudinalForce = calculateLongitudinalForce();

	// Calculate lateral force, cornering stiffness and max lateral force
	lateralForce = calculateLateralForce();

	// Aligning moment (force feedback)
	aligningForce = calculateAligningForce();
}

float Pacejka::calculateLongitudinalForce()
// Calculates longitudinal force
// From G. Genta's book, page 63
// Note that the units are inconsistent:
//   load is in kN
//   slipRatio is in percentage (=slipRatio*100=slipPercentage)
//   camber and slipAngle are in degrees
// Resulting forces are better defined:
//   longitudinalForce, lateralForce are in N
//   aligningForce     is in Nm
{
	float B, C , D, E;
	float longitudinalForce;
	float Sh,Sv;
	float uP;
	float loadSquared;

	// Calculate derived coefficients
	loadSquared = load * load;
	C = b0;
	uP = b1 * load + b2;
	D = uP * load;

	// Avoid div by 0
	if((C > -0.00001f && C < 0.00001f) || (D > -0.00001f && D < 0.00001f))
	{
		B = 99999.0f;
	}
	else
	{
		B = ((b3 * loadSquared + b4 * load) * expf(-b5 * load)) / (C * D);
	}

	E = b6 * loadSquared + b7 * load+b8;
	Sh = b9 * load + b10;
	Sv = 0;

	// Notice that product BCD is the longitudinal tire stiffness
	longitudinalStiffness = B * C * D;

	// Remember the max longitudinal force
	maxLongitudinalForce = D + Sv;

	// Calculate result force
	longitudinalForce = D * sinf(C * atanf(B * (1.0f - E) * (slipPercentage + Sh) + E * atanf(B * (slipPercentage + Sh)))) + Sv;

	return longitudinalForce;
}
float Pacejka::calculateLateralForce()
// Calculates lateral force
// Note that BCD is the cornering stiffness, and
// Sh and Sv account for ply steer and conicity forces
{
	float B, C, D, E;
	float lateralForce;
	float Sh,Sv;
	float uP;

	// Calculate derived coefficients
	C = a0;
	uP = a1 * load + a2;
	D = uP * load;
	E = a6 * load + a7;

	// Avoid div by 0
	if((C > -0.00001f && C < 0.00001f) || (D >- 0.00001f && D < 0.00001f))
	{
		B = 99999.0f;
	}
	else
	{
		if(a4 > -0.00001f && a4 < 0.00001f)
		{
			B = 99999.0f;
		}
		else
		{
			// Notice that product BCD is the lateral stiffness (=cornering)
			latStiffness = a3 * sinf(2 * atanf(load / a4)) * (1 -a5 * fabs(camber));
			B = latStiffness / (C * D);
		}
	}

	Sh = a8 * camber + a9 * load + a10;
	Sv = (a111 * load + a112) * camber * load + a12 * load + a13;

	// Remember maximum lateral force
	maxLateralForce = D + Sv;

	// Calculate result force
	lateralForce = D * sinf(C * atanf(B * (1.0f - E) * (slipAngle + Sh) + E * atanf(B * (slipAngle + Sh)))) + Sv;

	return lateralForce;
}

float Pacejka::calculateAligningForce()
{
	float aligningForce;
	float B, C, D, E, Sh, Sv;
	float loadSquared;

	// Calculate derived coefficients
	loadSquared = load * load;
	C = c0;
	D = c1 * loadSquared + c2 * load;
	E = (c7 * loadSquared + c8 * load + c9) * (1 - c10 * fabs(camber));
	
	if((C > -0.00001f && C < 0.00001f) || (D > -0.00001f && D < 0.00001f))
	{
		B = 99999.0f;
	}
	else
	{
		B = ((c3 * loadSquared + c4 * load) * (1 - c6 * fabs(camber)) * expf(-c5 * load)) / (C * D);
	}

	Sh = c11 * camber + c12 * load + c13;
	Sv = (c14 * loadSquared + c15 * load) * camber + c16 * load + c17;

	aligningForce = D * sinf(C * atanf(B * (1.0f - E) * (slipAngle + Sh) + E * atanf(B * (slipAngle + Sh)))) + Sv;

	return aligningForce;
}

float Pacejka::getMaxForce()
// Calculates maximum force that the tire can produce
// If the longitudinal and lateral force combined exceed this,
// a violation of the friction circle (max total tire force) is broken.
// In that case, reduce the lateral force, since the longitudinal force
// is more prominent (this simulates a locked braking wheel, which
// generates no lateral force anymore but does maximum longitudinal force).
{
	float uP = b1 * load + b2;

	return uP * load;
}