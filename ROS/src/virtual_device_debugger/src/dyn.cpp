//Using Chrono to simulate the vehicle.=
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChMarker.h"

#include <limits>
#include "pacejka.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
using namespace std;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool getTireForces(double load, double omega, double def_z, double roll_r, double v_wx, double v_wy, double v_wz, \
				   double F_vert, double F_lat, double F_long, double T_ali)
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
	
	const double k_t = 2.65e+05;
	const double d_t = 500.0;
	
	pacejka.setCamber(0.0f);
	pacejka.setLoad(load);

	//std::cout << "> Calculating Tire Contact (Friction) Forces.. ";
	
	//See the definitions about the tyre terms.
	double slipRatio = (omega*roll_r/std::max(fabs(v_wx), std::numeric_limits<double>::epsilon())\
	                    *sgn(v_wx) - 1)*100.0;
	double slipAngle = -atan(v_wy/v_wx);
	pacejka.setSlipRatio(slipRatio);
    pacejka.setSlipAngle(slipAngle);
	pacejka.calculate();
	
	F_vert = k_t * def_z + d_t * v_wz;
	F_long = pacejka.getLongitudinalForce();
	F_lat = pacejka.getLateralForce();
	T_ali = pacejka.getAligningForce();

    printf("F_long= %f    ; F_lat= %f  (N)    ; T_ali= %f  (Nm)\n", F_long, F_lat, T_ali);
	//std::cout << "done!" << std::endl;
	
	return 0;
}

class AgileVehicle {
    public:
        double torque[4] = {0.};                        //N.m according to definition
        double steer[4] = {0.};                         //RAD according to definition
        
        const double TW = 1.315;  //Track width
        const double WB = 1.520;  //Wheel base
        const double R_W = 0.315; //Wheel Radius
        std::shared_ptr<ChBody> chassis;
        std::shared_ptr<ChBody> motorFrameFL;
        std::shared_ptr<ChBody> wheelFL;
        std::shared_ptr<ChBody> motorFrameFR;
        std::shared_ptr<ChBody> wheelFR;
        std::shared_ptr<ChBody> motorFrameRL;
        std::shared_ptr<ChBody> wheelRL;
        std::shared_ptr<ChBody> motorFrameRR;
        std::shared_ptr<ChBody> wheelRR;
		std::shared_ptr<ChMarker> posFL;
		std::shared_ptr<ChMarker> posFR;
		std::shared_ptr<ChMarker> posRL;
		std::shared_ptr<ChMarker> posRR;
        std::shared_ptr<ChLinkLockRevolute> link_revoluteFL;
        std::shared_ptr<ChLinkMateGeneric> suspFL;
        std::shared_ptr<ChLinkLockRevolute> link_revoluteFR;
        std::shared_ptr<ChLinkMateGeneric> suspFR;
        std::shared_ptr<ChLinkLockRevolute> link_revoluteRL;
        std::shared_ptr<ChLinkMateGeneric> suspRL;
        std::shared_ptr<ChLinkLockRevolute> link_revoluteRR;
        std::shared_ptr<ChLinkMateGeneric> suspRR;
        
        const double k_S = 110.0e+3;
        const double k_C = 15.0e+3;
        const double k_R1 = 2000.0;
        const double k_R2 = 20000.0;
        const double d_S = 200.0;
        const double d_C = 2000.0;
        const double d_R1 = 100.0;
        const double d_R2 = 100.0;
	    double alpha_1[4] , alpha_2[4];
        // Constructing the suspension by defining force.
        void setSuspensionForce (ChSystemNSC* msystem,  // contains all bodies
                                 std::shared_ptr<ChBody> chassis, std::shared_ptr<ChMarker> ref,
                                 std::shared_ptr<ChBody> motorFrame, int i) {
            chassis->Empty_forces_accumulators ();
			motorFrame->Empty_forces_accumulators ();
            double C1 = cos(alpha_1[i]);
		    double S1 = sin(alpha_1[i]);
		    double C2 = cos(alpha_2[i]);
		    double S2 = sin(alpha_2[i]);
			const ChVector<double> X0 = (0.0, 0.0, -0.30);
            ChMatrix33<> K1;
            ChMatrix33<> d1;
            K1(0,0) = 2*k_S*C1*C1+k_R2;
		    K1(0,1) = 0.0;
		    K1(0,2) = 0.0;
		    K1(1,1) = k_C*C2*C2+2*k_R1;
		    K1(1,2) = k_C*C2*S2;
		    K1(2,2) = k_C*S2*S2+2*k_S*S1*S1;
		    d1(0,0) = 2*d_S*C1*C1+d_R2;
		    d1(0,1) = 0.0;
		    d1(0,2) = 0.0;
		    d1(1,1) = d_C*C2*C2+2*d_R1;
		    d1(1,2) = d_C*C2*S2;
		    d1(2,2) = d_C*S2*S2+2*d_S*S1*S1;
		    //TO BE POPULATED.
			//In Absolute Frame
			ChVector<> V_r = (motorFrame->GetPos_dt())-(ref->GetPos_dt());
			ChVector<> X_r = (motorFrame->GetPos())-(ref->GetPos())-X0;
			ChMatrix33<> RotT = motorFrame->GetA();
			//ChMatrix33<double> RotB = ref->GetA();
			ChMatrix33<> tmp1, tmp2;
			tmp1.MatrMultiply(RotT,K1);
			tmp1.MatrMultiplyT(tmp1,RotT);
			tmp2.MatrMultiply(RotT,d1);
			tmp2.MatrMultiplyT(tmp2,RotT);
			ChVector<> force = 	tmp1*X_r + tmp2*V_r;
			chassis->Accumulate_force(force, ref->GetPos(), false);
			motorFrame->Accumulate_force(-force, motorFrame->GetPos(), false);
        }
        
        void setTyreForce (ChSystemNSC* msystem,        // contains all bodies
                           std::shared_ptr<ChBody> wheel, int i) {
            wheel->Empty_forces_accumulators();
			ChVector<> load = wheel->GetContactForce();
			ChVector<> omega = wheel->GetWvel_loc();
			
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
        
        AgileVehicle (ChSystemNSC& my_system,           
                      ISceneManager* msceneManager,  
                      IVideoDriver* mdriver) {
            // Texture for wheels
            auto texture = std::make_shared<ChTexture>();
            texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            
            // --- The car body ---
            chassis = std::make_shared<ChBodyEasyBox>(1.75, 1.5, 0.5, 294.0, true, true);
            chassis->SetPos(ChVector<>(0, 0, 1.1));
            chassis->SetBodyFixed(false);
            my_system.AddBody(chassis);
            
            // --- Motor FL ---
            motorFrameFL = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameFL->SetPos(ChVector<>(WB/2.0, TW/2.0-0.1, R_W/2.0));
            motorFrameFL->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            motorFrameFL->SetBodyFixed(false);
            my_system.AddBody(motorFrameFL);
            
            // --- Motor FR ---
            motorFrameFR = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameFR->SetPos(ChVector<>(WB/2.0, -TW/2.0+0.1, R_W/2.0));
            motorFrameFR->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            motorFrameFR->SetBodyFixed(false);
            my_system.AddBody(motorFrameFR);
            
            // --- Motor RL ---
            motorFrameRL = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameRL->SetPos(ChVector<>(-WB/2.0, TW/2.0-0.1, R_W/2.0));
            motorFrameRL->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            motorFrameRL->SetBodyFixed(false);
            my_system.AddBody(motorFrameRL);
            
            // --- Motor RR ---
            motorFrameRR = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0+0.1, R_W/2.0));
            motorFrameRR->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            motorFrameRR->SetBodyFixed(false);
            my_system.AddBody(motorFrameRR);
            
            // --- Wheels ---
            wheelFL = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelFL->SetPos(ChVector<>(WB/2.0, TW/2.0, R_W/2.0));
            wheelFL->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
			wheelFL->SetBodyFixed(false);
            wheelFL->AddAsset(texture);
            my_system.AddBody(wheelFL);
            
            // --- Wheels ---
            wheelFR = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelFR->SetPos(ChVector<>(WB/2.0, -TW/2.0, R_W/2.0));
            wheelFR->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
			wheelFR->SetBodyFixed(false);
            wheelFR->AddAsset(texture);
            my_system.AddBody(wheelFR);
            
            // --- Wheels ---
            wheelRL = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelRL->SetPos(ChVector<>(-WB/2.0, TW/2.0, R_W/2.0));
            wheelRL->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
			wheelRL->SetBodyFixed(false);
            wheelRL->AddAsset(texture);
            my_system.AddBody(wheelRL);
            
            // --- Wheels ---
            wheelRR = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0, R_W/2.0));
            wheelRR->SetRot(Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
			wheelRR->SetBodyFixed(false);
            wheelRR->AddAsset(texture);
            my_system.AddBody(wheelRR);
            
            // R-joints between wheel and motor frames
            link_revoluteFL = std::make_shared<ChLinkLockRevolute>();
            link_revoluteFL->Initialize(wheelFL, motorFrameFL, \
                             ChCoordsys<>(ChVector<>(WB/2.0, TW/2.0-0.05, R_W/2.0), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteFL);
            
            link_revoluteFR = std::make_shared<ChLinkLockRevolute>();
            link_revoluteFR->Initialize(wheelFR, motorFrameFR, \
                             ChCoordsys<>(ChVector<>(WB/2.0, -TW/2.0+0.05, R_W/2.0), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteFR);
            
            link_revoluteRL = std::make_shared<ChLinkLockRevolute>();
            link_revoluteRL->Initialize(wheelRL, motorFrameRL, \
                             ChCoordsys<>(ChVector<>(-WB/2.0, TW/2.0-0.05, R_W/2.0), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteRL);
            
            link_revoluteRR = std::make_shared<ChLinkLockRevolute>();
            link_revoluteRR->Initialize(wheelRR, motorFrameRR, \
                             ChCoordsys<>(ChVector<>(-WB/2.0, -TW/2.0+0.05, R_W/2.0), Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteRR);
            
			posFL = std::make_shared<ChMarker>();
			chassis->AddMarker(posFL);
			posFL->SetPos(ChVector<>(WB/2.0, TW/2.0, R_W/2.0+0.3));
			posFL->Impose_Abs_Coord(chassis->GetCoord());
			
			posFR = std::make_shared<ChMarker>();
			chassis->AddMarker(posFR);
			posFR->SetPos(ChVector<>(WB/2.0, -TW/2.0, R_W/2.0+0.3));
			posFR->Impose_Abs_Coord(chassis->GetCoord());
			
			posRL = std::make_shared<ChMarker>();
			chassis->AddMarker(posRL);
			posRL->SetPos(ChVector<>(-WB/2.0, TW/2.0, R_W/2.0+0.3));
			posRL->Impose_Abs_Coord(chassis->GetCoord());
			
			posRR = std::make_shared<ChMarker>();
			chassis->AddMarker(posRR);
			posRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0, R_W/2.0+0.3));
			posRR->Impose_Abs_Coord(chassis->GetCoord());
			
            suspFL = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> susp_abs_FL(posFL->GetPos());
            suspFL->Initialize(wheelFL,             // the 1st body to connect
                               chassis,             // the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_FL,         // the link reference attached to 1st body
                               susp_abs_FL);       // the link reference attached to 2nd body
            my_system.Add(suspFL);
            
            suspFR = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> susp_abs_FR(posFR->GetPos());
            suspFR->Initialize(wheelFR,             // the 1st body to connect
                               chassis,             // the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_FR,         // the link reference attached to 1st body
                               susp_abs_FR);        // the link reference attached to 2nd body
            my_system.Add(suspFR);
            
            suspRL = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> susp_abs_RL(posRL->GetPos());
            suspRL->Initialize(wheelRL,        		// the 1st body to connect
                               chassis,           	// the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_RL,         // the link reference attached to 1st body
                               susp_abs_RL);        // the link reference attached to 2nd body
            my_system.Add(suspRL);
            
            suspRR = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> susp_abs_RR(posRR->GetPos());
            suspRR->Initialize(wheelRR,        		// the 1st body to connect
                               chassis,           	// the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_RR,         // the link reference attached to 1st body
                               susp_abs_RR);        // the link reference attached to 2nd body
            my_system.Add(suspRR);
        }
        
        ~AgileVehicle () {
            ChSystem* mysystem = wheelFL->GetSystem();  // trick to get the system here
            // When a ChBodySceneNode is removed via ->remove() from Irrlicht 3D scene manager,
            // it is also automatically removed from the ChSystemNSC (the ChSystemNSC::RemoveBody() is
            // automatically called at Irrlicht node deletion - see ChBodySceneNode.h ).
            // For links, just remove them from the ChSystemNSC using ChSystemNSC::RemoveLink()
            mysystem->RemoveLink(link_revoluteFL);
            mysystem->RemoveLink(link_revoluteFR);
            mysystem->RemoveLink(link_revoluteRL);
            mysystem->RemoveLink(link_revoluteRR);
            mysystem->Remove(suspFL);
            mysystem->Remove(suspFR);
            mysystem->Remove(suspRL);
            mysystem->Remove(suspRR);
        }
        
        
};
    
int main_undone(int argc, char* argv[]) {
    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //
    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));
    
    // 2.- Create the Irrlicht visualization.
    ChIrrApp application(&my_system, L"Simple vehicle suspension", core::dimension2d<u32>(640, 480), false);
    //ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6));
    application.SetTryRealtime(true);
    
    // 3- Create the rigid bodies of the simpified car suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.
    // ..the world
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("blu.png"));
    
    auto ground = std::make_shared<ChBody>();
    my_system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetName("ground");
    ground->SetBodyFixed(true);
    
    for (int i = 0; i < 6; i++) {
        auto my_obstacle = std::make_shared<ChBodyEasyBox>(1, 0.1, 0.5, 60.0, true, true);
        my_obstacle->SetPos(ChVector<>(20 * ChRandom(), 2, 20 * ChRandom()));
        my_obstacle->SetMass(3);
        my_system.AddBody(my_obstacle);
    }
    
    AgileVehicle* AgileV = new AgileVehicle(my_system, application.GetSceneManager(), application.GetVideoDriver());
    
    ChRealtimeStepTimer m_realtime_timer;
    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();
        // Draw an XZ grid at the global origin to add in visualization.
        ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);
        // .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
        application.GetIGUIEnvironment()->drawAll();
        // .. draw the distance constraints (the massless rods) as simplified lines
        auto iterlink = my_system.Get_linklist()->begin();
        while (iterlink != my_system.Get_linklist()->end()) {
            if (auto mylinkdis = std::dynamic_pointer_cast<ChLinkDistance>(*iterlink))
                ChIrrTools::drawSegment(application.GetVideoDriver(), mylinkdis->GetEndPoint1Abs(), mylinkdis->GetEndPoint2Abs(),
                                        video::SColor(255, 0, 20, 0), true);
            iterlink++;
        }
        
        //TO BE POPULATED.
        
        {
            //Add Custom Forces to support the system
            AgileV->setSuspensionForce(&my_system, AgileV->chassis, AgileV->posFL, AgileV->motorFrameFL, 0);
            AgileV->setSuspensionForce(&my_system, AgileV->chassis, AgileV->posFR, AgileV->motorFrameFR, 1);
            AgileV->setSuspensionForce(&my_system, AgileV->chassis, AgileV->posRL, AgileV->motorFrameRL, 2);
            AgileV->setSuspensionForce(&my_system, AgileV->chassis, AgileV->posRR, AgileV->motorFrameRR, 3);
            
            AgileV->setTyreForce(&my_system, AgileV->wheelFL, 0);
            AgileV->setTyreForce(&my_system, AgileV->wheelFR, 1);
            AgileV->setTyreForce(&my_system, AgileV->wheelRL, 2);
            AgileV->setTyreForce(&my_system, AgileV->wheelRR, 3);
        }
        
        // HERE CHRONO INTEGRATION IS PERFORMED: THE
        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
        // STEP:
        my_system.DoStepDynamics(m_realtime_timer.SuggestSimulationStep(0.025));
        // Irrlicht must finish drawing the frame
        application.EndScene();
    }
    
    if (AgileV)
        delete AgileV;
    return 0;
}        
