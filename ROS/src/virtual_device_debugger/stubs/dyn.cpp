#include "Simbody.h"
#include <iostream>
using namespace SimTK;
using std::cout; using std::endl; using std::cin;

#define ANIMATE

class UserInputHandler : public PeriodicEventHandler {
public:
    UserInputHandler(Visualizer::InputSilo& silo, 
                     const Motion::Steady&  motor, 
                     Real                   interval); 
    void handleEvent(State& state, Real accuracy,
                     bool& shouldTerminate) const override;
private:
    Visualizer::InputSilo& m_silo;
    Motion::Steady         m_motor;
};

class SuspensionForce : public Force::Custom::Implementation {
public:
    SuspensionForce(SimbodyMatterSubsystem& matter) : matter(matter) {}
    void calcForce(const State& state, Vector_<SpatialVec>& bodyForces, \
                   Vector_<Vec3>& particleForces, Vector& mobilityForces) const {
        for (MobilizedBodyIndex i(0); i<matter.getNumBodies(); i++) {
            const MobilizedBody& body1 = matter.getMobilizedBody(i);
            for (MobilizedBodyIndex j(0); j < i; j++) {
                const MobilizedBody& body2 = matter.getMobilizedBody(j);
                Vec3 r = body1.getBodyOriginLocation(state) - body2.getBodyOriginLocation(state);
                Vec3 v = body1.getBodyOriginVelocity(state) - body2.getBodyOriginVelocity(state);
                Vec3 force;
            }
        }
    }
    Real calcPotentialEnergy(const State& state) const {
        double energy = 0.0;
        for (MobilizedBodyIndex i(0); i < matter.getNumBodies(); i++) {
            const MobilizedBody& body1 = matter.getMobilizedBody(i);
            for (MobilizedBodyIndex j(0); j < i; j++) {
                const MobilizedBody& body2 = matter.getMobilizedBody(j);
                Vec3 r = body1.getBodyOriginLocation(state)-
                body2.getBodyOriginLocation(state);
                energy -= 1.0/r.norm();
            }
        }
        return energy;
    }
private:
    SimbodyMatterSubsystem& matter;
};

const Real mu_s = 0.7;       // Friction coefficients.
const Real mu_d = 0.5;
const Real mu_v = 0;
const Real transitionVelocity = 1e-3; // slide->stick velocity

// Rubber for SpeedBump
const Real rubber_density = 1100.;  // kg/m^3
const Real rubber_young   = 0.01e9/10; // pascals (N/m)
const Real rubber_poisson = 0.5;    // ratio
const Real rubber_planestrain = 
    ContactMaterial::calcPlaneStrainStiffness(rubber_young,rubber_poisson);
const Real rubber_dissipation = /*0.005*/10;

const ContactMaterial rubber(rubber_planestrain,rubber_dissipation,
                               mu_s,mu_d,mu_v);

// Concrete for ground
const Real concrete_density = 2300.;  // kg/m^3
const Real concrete_young   = 25e9;  // pascals (N/m)
const Real concrete_poisson = 0.15;    // ratio
const Real concrete_planestrain = 
    ContactMaterial::calcPlaneStrainStiffness(concrete_young,concrete_poisson);
const Real concrete_dissipation = 0.005;

const ContactMaterial concrete(concrete_planestrain,concrete_dissipation,
                               mu_s,mu_d,mu_v);

int main(){
    try{
    MultibodySystem system;
    SimbodyMatterSubsystem matter(system);
    GeneralForceSubsystem forced(system);
    ContactTrackerSubsystem tracker(system);
    CompliantContractSubsystem contract(system);
    contact.setTransitionVelocity(0.001);
    Force::Gravity(forces, matter, -YAxis, 9.81);
    
    // Set up visualization and ask for a frame every 1/30 second.
    Visualizer viz(system);
    viz.setShowSimTime(true); viz.setShowFrameRate(true);
    Visualizer::InputSilo* silo = new Visualizer::InputSilo();
    viz.addInputListener(silo);   
    #ifdef ANIMATE
    system.addEventReporter(new Visualizer::Reporter(viz, 1./30));
    #endif
    DecorativeText help("Any input to start; ESC to quit");
    help.setIsScreenText(true);
    viz.addDecoration(MobilizedBodyIndex(0),Vec3(0),help);
    matter.setShowDefaultGeometry(false);
    
    // Add the Ground contact geometry. Contact half space has -XAxis normal
    // (right hand wall) so we have to rotate.
    MobilizedBody& Ground = matter.updGround(); // Nicer name for Ground.
    Ground.updBody().addContactSurface(Transform(YtoX,Vec3(0)),
        ContactSurface(ContactGeometry::HalfSpace(),concrete));

    // Add some speed bumps.
    const int NBumps = 2; const Vec3 BumpShape(.8,0.2,2);
    for (int i=0; i < NBumps; ++i) {
        const Real x = -2*(i+1);
        Ground.updBody().addContactSurface(Vec3(x,0,0),
            ContactSurface(ContactGeometry::Ellipsoid(BumpShape), rubber));
        Ground.updBody().addDecoration(Vec3(x,0,0),
            DecorativeEllipsoid(BumpShape).setColor(Gray).setResolution(3));
    }
    
    //Chassis
    const Vec3 chassisDims(1.75*0.5,0.25,1.5*0.5);
    const Real chassisMass = 400;
    const Real unsprungMass = 40;
    
    const Real TW = 1.315;  //Track width
    const Real WB = 1.520;  //Wheel base
    const Real R_W = 0.315; //Wheel Radius
    const Vec3 chassisCM(0,0,0);
    Body::Rigid chassisInfo(MassProperties(chassisMass,chassisCM,
        UnitInertia::brick(chassisDims).shiftFromCentroidInPlace(-chassisCM)));
    chassisInfo.addDecoration(Vec3(0), DecorativeBrick(chassisDims).setColor(Cyan));
    MobilizedBody::Free chassis(Ground,Vec3(0,1.0,0), chassisInfo,Vec3(0));
    
    Array_<Body::Rigid, 4> MotorFrame;
    Array_<SuspensionForce, 4> Suspension;
    Array_<TyreForce, 4> Tyres;
    
    MotorFrame.assign(Body::Rigid);
    Suspension.assign(SuspensionForce);
    Tyres.assign(TyreForce);
    
//Conversions
//X --> X
//Y -->-Z
//Z --> Y

    const WheelMarkers[4][3] = {WB/2.0, -0.25, -TW/2.0, \
                                WB/2.0, -0.25,  TW/2.0, \
                               -WB/2.0, -0.25, -TW/2.0, \
                               -WB/2.0, -0.25,  TW/2.0};
                               
    const MotorMarkers[4][3] = {WB/2.0, -0.25, -TW/2.0+0.15, \
                                WB/2.0, -0.25,  TW/2.0-0.15, \
                               -WB/2.0, -0.25, -TW/2.0+0.15, \
                               -WB/2.0, -0.25,  TW/2.0-0.15};
    
    for (int i=0; i<4; i++){
        const Vec3 MotorCenter(MotorMarkers[i][0], MotorMarkers[i][1], MotorMarkers[i][2]);
        MotorFrame[i](MassProperties(unsprungMass*0.4, MotorCenter, UnitInertia::cylinderAlongZ(0.18, 0.30)));
        MotorFrame[i].addDecoration(Vec3(0, 0, 1), DecorativeCylinder(0.18, 0.30).setColor(Red));
        
    }
