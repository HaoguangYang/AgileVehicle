//Using Chrono to simulate the vehicle.

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChTimer.h"

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

class AgileVehicle {
    public:
        double[4] torque = {0.};                        //N.m according to definition
        double[4] steer = {0.};                         //RAD according to definition
        
        const Real TW = 1.315;  //Track width
        const Real WB = 1.520;  //Wheel base
        const Real R_W = 0.315; //Wheel Radius
        std::shared_ptr<ChBody> chassis;
        std::shared_ptr<ChBody> motorFrameFL;
        std::shared_ptr<ChBody> wheelFL;
        std::shared_ptr<ChBody> motorFrameFR;
        std::shared_ptr<ChBody> wheelFR;
        std::shared_ptr<ChBody> motorFrameRL;
        std::shared_ptr<ChBody> wheelRL;
        std::shared_ptr<ChBody> motorFrameRR;
        std::shared_ptr<ChBody> wheelRR;
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
                                 std::shared_ptr<ChBody>* chassis,
                                 std::shared_ptr<ChBody>* motorFrame, int i) {
            chassis->Empty_forces_accumulators();
            double C1 = cos(alpha_1[i]);
		    double S1 = sin(alpha_1[i]);
		    double C2 = cos(alpha_2[i]);
		    double S2 = sin(alpha_2[i]);
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
        }
        
        void setTyreForce (ChSystemNSC* msystem,        // contains all bodies
                           std::shared_ptr<ChBody> wheel, int i) {
            
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
            motorFrameFL->SetPos(ChVector<>(WB/2.0, TW/2.0-0.15, R_W/2.0));
            motorFrameFL->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            //motorFrameFL->SetBodyFixed(false);
            my_system.AddBody(motorFrameFL);
            
            // --- Motor FR ---
            motorFrameFR = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameFR->SetPos(ChVector<>(WB/2.0, -TW/2.0+0.15, R_W/2.0));
            motorFrameFR->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            //motorFrameFR->SetBodyFixed(false);
            my_system.AddBody(motorFrameFR);
            
            // --- Motor RL ---
            motorFrameRL = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameRL->SetPos(ChVector<>(-WB/2.0, TW/2.0-0.15, R_W/2.0));
            motorFrameRL->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            //motorFrameRL->SetBodyFixed(false);
            my_system.AddBody(motorFrameRL);
            
            // --- Motor RR ---
            motorFrameRR = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0+0.15, R_W/2.0));
            motorFrameRR->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            //motorFrameRR->SetBodyFixed(false);
            my_system.AddBody(motorFrameRR);
            
            // --- Wheels ---
            wheelFL = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelFL->SetPos(ChVector<>(WB/2.0, TW/2.0, R_W/2.0));
            wheelFL->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            wheelFL->AddAsset(texture);
            my_system.AddBody(wheelFL);
            
            // --- Wheels ---
            wheelFR = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelFR->SetPos(ChVector<>(WB/2.0, -TW/2.0, R_W/2.0));
            wheelFR->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            wheelFR->AddAsset(texture);
            my_system.AddBody(wheelFR);
            
            // --- Wheels ---
            wheelRL = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelRL->SetPos(ChVector<>(-WB/2.0, TW/2.0, R_W/2.0));
            wheelRL->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            wheelRL->AddAsset(texture);
            my_system.AddBody(wheelRL);
            
            // --- Wheels ---
            wheelRR = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0, R_W/2.0));
            wheelRR->SetRot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
            wheelRR->AddAsset(texture);
            my_system.AddBody(wheelRR);
            
            // R-joints between wheel and motor frames
            link_revoluteFL = std::make_shared<ChLinkLockRevolute>();
            link_revoluteFL->Initialize(wheelFL, motorFrameFL, \
                             ChCoordsys<>(ChVector<>(WB/2.0, TW/2.0-0.15, R_W/2.0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteFL);
            
            link_revoluteFR = std::make_shared<ChLinkLockRevolute>();
            link_revoluteFR->Initialize(wheelFR, motorFrameFR, \
                             ChCoordsys<>(ChVector<>(WB/2.0, -TW/2.0+0.15, R_W/2.0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteFR);
            
            link_revoluteRL = std::make_shared<ChLinkLockRevolute>();
            link_revoluteRL->Initialize(wheelRL, motorFrameRL, \
                             ChCoordsys<>(ChVector<>(-WB/2.0, TW/2.0-0.15, R_W/2.0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteRL);
            
            link_revoluteRR = std::make_shared<ChLinkLockRevolute>();
            link_revoluteRR->Initialize(wheelRR, motorFrameRR, \
                             ChCoordsys<>(ChVector<>(-WB/2.0, -TW/2.0+0.15, R_W/2.0), chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
            my_system.AddLink(link_revoluteRR);
            
            suspFL = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> link_position_abs(ChVector<>(WB/2.0, TW/2.0, R_W/2.0+0.3));
            sphericalLink->Initialize(wheelFL,        // the 1st body to connect
                                      chassis,           // the 2nd body to connect
                                      false,               // the two following frames are in absolute, not relative, coords.
                                      link_position_abs,   // the link reference attached to 1st body
                                      link_position_abs);  // the link reference attached to 2nd body
            my_system.Add(suspFL);
            
            suspFR = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> link_position_abs(ChVector<>(WB/2.0, -TW/2.0, R_W/2.0+0.3));
            sphericalLink->Initialize(wheelFR,        // the 1st body to connect
                                      chassis,           // the 2nd body to connect
                                      false,               // the two following frames are in absolute, not relative, coords.
                                      link_position_abs,   // the link reference attached to 1st body
                                      link_position_abs);  // the link reference attached to 2nd body
            my_system.Add(suspFR);
            
            suspRL = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> link_position_abs(ChVector<>(-WB/2.0, TW/2.0, R_W/2.0+0.3));
            sphericalLink->Initialize(wheelRL,        // the 1st body to connect
                                      chassis,           // the 2nd body to connect
                                      false,               // the two following frames are in absolute, not relative, coords.
                                      link_position_abs,   // the link reference attached to 1st body
                                      link_position_abs);  // the link reference attached to 2nd body
            my_system.Add(suspRL);
            
            suspRR = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, false);  // x,y,z,Rx,Ry,Rz constrains
            ChFrame<> link_position_abs(ChVector<>(-WB/2.0, -TW/2.0, R_W/2.0+0.3));
            sphericalLink->Initialize(wheelRR,        // the 1st body to connect
                                      chassis,           // the 2nd body to connect
                                      false,               // the two following frames are in absolute, not relative, coords.
                                      link_position_abs,   // the link reference attached to 1st body
                                      link_position_abs);  // the link reference attached to 2nd body
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
        
        
}
    
int main(int argc, char* argv[]) {
    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //
    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;
    system.Set_G_acc(ChVector<>(0, 0, -9.81));
    
    // 2.- Create the Irrlicht visualization.
    ChIrrApp application(&my_system, L"Simple vehicle suspension", core::dimension2d<u32>(640, 480), false);
    //ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 0, -6));
    application->SetTryRealtime(true);
    
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
        ChIrrTools::drawGrid(application->GetVideoDriver(), 1, 1, 20, 20,
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
            setSuspensionForce(&my_system, &chassis, &motorFrameFL, 0);
            setSuspensionForce(&my_system, &chassis, &motorFrameFR, 1);
            setSuspensionForce(&my_system, &chassis, &motorFrameRL, 2);
            setSuspensionForce(&my_system, &chassis, &motorFrameRR, 3);
            
            setTyreForce(&my_system, &wheelFL, 0);
            setTyreForce(&my_system, &wheelFR, 1);
            setTyreForce(&my_system, &wheelRL, 2);
            setTyreForce(&my_system, &wheelRR, 3);
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
