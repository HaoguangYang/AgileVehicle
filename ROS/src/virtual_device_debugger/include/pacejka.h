#ifndef PACEJKA_H_INCLUDED
#define PACEJKA_H_INCLUDED

// Based on Racer code
// See http://www.racer.nl/reference/pacejka.htm for info

class Pacejka
{
public:
	// Lateral force coefficients
	float a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a111, a112, a12, a13;

	// Longitudinal force coefficients
	float b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10;

	// Aligning moment coefficients
	float c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17;

protected:
	// Input parameters
	float camber;                // Angle of tire vs. surface (in degrees)
	float slipAngle;             // Slip angle (in degrees)
	float slipPercentage;        // Percentage slip ratio (in %)
	float load;                  // Normal force (in kN)
	
	// Output
	float longitudinalForce;
	float lateralForce;
	float aligningForce;
	float longitudinalStiffness; // Longitudinal tire stiffness
	float latStiffness;          // Lateral or cornering 
	float maxLongitudinalForce;  // Max available tire longitudinal force (friction ellipse)
	float maxLateralForce;       // Max available tire lateral force (friction ellipse)

public:
	Pacejka();

	void setCamber(float camberDegrees) { camber = camberDegrees; }
	void setSlipAngle(float slipAngleDegrees) { slipAngle = slipAngleDegrees; }
	void setSlipRatio(float slipRatio) { slipPercentage = slipRatio * 100.0f; }
	void setLoad(float loadForce) { load = loadForce / 1000.0f; }

	void calculate();

protected:
	float calculateLongitudinalForce();
	float calculateLateralForce();
	float calculateAligningForce();

public:
	float getLongitudinalForce() { return longitudinalForce; }
	float getLateralForce() { return lateralForce; }
	float getAligningForce() { return aligningForce; }

	void setLongitudinalForce(float magnitude) { longitudinalForce = magnitude; }
	void setLateralForce(float magnitude) { lateralForce = magnitude; }
	void setAligningForce(float magnitude) { aligningForce = magnitude; }

	float getMaxForce();
	float getMaxLongitudinalForce() { return maxLongitudinalForce; }
	float getMaxLateralForce() { return maxLateralForce; }

	void setMaxLongitudinalForce(float magnitude) { maxLongitudinalForce = magnitude; }
	void setMaxLateralForce(float magnitude) { maxLateralForce = magnitude; }

	float getLongitudinalStiffness() { return longitudinalStiffness; }
	float getLateralStiffness() { return latStiffness; }
};

#endif