/*
 * Using Chrono to simulate the vehicle dynamics.
 * TODO: VEHICLE TIPS OVER !
 * This code is released under GPL-3.0 License.
 *
 * Author: Haoguang Yang
 * July 1, 2017
 *
 */

//
#include "dyn.h"

bool no_quit = true;

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
	
	if (def_z > -0.0005)
	    F_vert = k_t * def_z + d_t * v_wz;
	else
	    F_vert = 0.;
	F_long = pacejka.getLongitudinalForce();
	F_lat = pacejka.getLateralForce();
	T_ali = pacejka.getAligningForce();

    printf("F_long= %f    ; F_lat= %f  (N)    ; T_ali= %f  (Nm)\n", F_long, F_lat, T_ali);
	//std::cout << "done!" << std::endl;
	
	return 0;
}

class AgileVehicle {
    public:
        double torque[4] = {0., 0., 0., 0.};                        //N.m according to definition
        double steer[4] = {0., 0., 0., 0.};                         //RAD according to definition
        
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
	    double alpha_1[4] = {1.4, 1.4, 1.4, 1.4};
	    double alpha_2[4] = {1.3, 1.3, 1.3, 1.3};
	    double Torque[4] = {0.,0., 0., 0.};
        // Constructing the suspension by defining force.
        void setSuspensionForce (std::shared_ptr<ChBody> chassis, std::shared_ptr<ChMarker> ref,
                                 std::shared_ptr<ChBody> motorFrame, std::shared_ptr<ChBody> wheel, int i, ChIrrApp* application) {
            chassis->Empty_forces_accumulators ();
			motorFrame->Empty_forces_accumulators ();
            double C1 = cos(alpha_1[i]);
		    double S1 = sin(alpha_1[i]);
		    double C2 = cos(alpha_2[i]);
		    double S2 = sin(alpha_2[i]);
			//ChFrame<> X0(ChVector<double>(0.0, 0.0, -0.30));
            ChMatrix33<> K1;
            ChMatrix33<> d1;
            K1(0,0) = 2*k_S*C1*C1+k_R2;
		    K1(0,1) = 0.0;
		    K1(0,2) = 0.0;
		    K1(1,0) = 0.0;
		    K1(1,1) = k_C*C2*C2+2*k_R1;
		    K1(1,2) = k_C*C2*S2;
		    K1(2,0) = 0.0;
		    K1(2,1) = K1(1,2);
		    K1(2,2) = k_C*S2*S2+2*k_S*S1*S1;
		    d1(0,0) = 2*d_S*C1*C1+d_R2;
		    d1(0,1) = 0.0;
		    d1(0,2) = 0.0;
		    d1(1,0) = 0.0;
		    d1(1,1) = d_C*C2*C2+2*d_R1;
		    d1(1,2) = d_C*C2*S2;
		    d1(2,0) = 0.0;
		    d1(2,1) = d1(1,2);
		    d1(2,2) = d_C*S2*S2+2*d_S*S1*S1;
		    
			//In Absolute Frame
			//Seems like it is correct now...
			ChVector<> V_r = (motorFrame->GetCoord()).TransformDirectionParentToLocal(motorFrame->GetPos_dt()-(ref->GetAbsFrame()).GetPos_dt());
			ChVector<> X_r = (motorFrame->GetCoord()).TransformDirectionParentToLocal(wheel->GetPos()-(ref->GetAbsFrame()).GetPos());
			//printf ("%f %f %f %f %f %f \n",V_r.x(), V_r.y(), V_r.z(), X_r.x(), X_r.y(), X_r.z());
			ChVector<> force = K1*X_r + d1*V_r;
			force = (motorFrame->GetCoord()).TransformDirectionLocalToParent(force);
			//printf ("%f %f %f\n",force.x(), force.y(), force.z());
			chassis->Accumulate_force(force, motorFrame->GetPos(), false);
			motorFrame->Accumulate_force(-force, motorFrame->GetPos(), false);
			ChIrrTools::drawSegment(application->GetVideoDriver(), motorFrame->GetPos(), motorFrame->GetPos()-force*0.003, video::SColor(255, 0, 20, 0), true);
        }
        
        void setTyreForce (ChSystemNSC* msystem,        // contains all bodies
                           std::shared_ptr<ChBody> chassis,	//should be std::shared_ptr<ChMarker> ref ?
                           std::shared_ptr<ChBody> wheel, int i) {
            wheel->Empty_forces_accumulators();
			ChVector<> load = wheel->GetContactForce();
			ChVector<> omega = wheel->GetWvel_loc();
			double def_z = std::max(R_W-(wheel->GetPos()).z(), -0.0005);
			//TODO: TO BE POPULATED
			AngSpeed[i] = omega.y();
			BLDC_model(ctrlVolt[i], AngSpeed[i], Torque[i]);
			ChVector<> v_w = ((wheel->GetFrame_COG_to_abs())>>(chassis->GetFrame_COG_to_abs())).GetPos_dt();	//TODO
			double C = cos(angle_real[i]/180.0*M_PI);
			double S = sin(angle_real[i]/180.0*M_PI);
			double v_wx = v_w.x()*C+v_w.y()*S;
			double v_wy = v_w.y()*C-v_w.x()*S;
			double v_wz = v_w.z();
			double F_vert, F_lat, F_long, T_ali;
			getTireForces(load.z(), omega.y(), def_z, R_W-def_z, v_wx, v_wy, v_wz, F_vert, F_lat, F_long, T_ali);
			wheel-> Accumulate_force(ChVector<>(F_vert, F_lat, F_long), ChVector<>(0., 0., 0.), true);
			wheel-> Accumulate_torque(ChVector<>(0., Torque[i]-F_long*(R_W-def_z), T_ali), true);
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
        
        void TestSteering (std::shared_ptr<ChBody> motorFrame,
                           double Omega) {
            motorFrame -> SetWvel_par(ChVector<>(0., 0., Omega));	//TODO
        }
        
        void TestDriving (std::shared_ptr<ChBody> Wheel,
                          double Omega) {
            Wheel -> SetWvel_par(ChVector<>(Omega, 0., 0.));	//TODO
        }
        
        AgileVehicle (ChSystemNSC& my_system,           
                      ISceneManager* msceneManager,  
                      IVideoDriver* mdriver) {
            // Texture for wheels
            auto texture = std::make_shared<ChTexture>();
            texture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            
            // --- The car body ---
            chassis = std::make_shared<ChBodyEasyBox>(1.75, 1.5, 0.5, 294.0, true, true);
            chassis->SetName("Chassis");
            chassis->SetPos(ChVector<>(0, 0, 1.1));
            chassis->SetBodyFixed(false);
            my_system.AddBody(chassis);
            
            // --- Motor FL ---
            motorFrameFL = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameFL->SetName("motorFrameFL");
            motorFrameFL->SetPos(ChVector<>(WB/2.0, TW/2.0-0.15, R_W));
            motorFrameFL->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_Y));
            motorFrameFL->SetBodyFixed(false);
            my_system.AddBody(motorFrameFL);
            
            // --- Motor FR ---
            motorFrameFR = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameFR->SetName("motorFrameFR");
            motorFrameFR->SetPos(ChVector<>(WB/2.0, -TW/2.0+0.15, R_W));
            motorFrameFR->SetRot(Q_from_AngAxis(-CH_C_PI_2, VECT_Y));
            motorFrameFR->SetBodyFixed(false);
            my_system.AddBody(motorFrameFR);
            
            // --- Motor RL ---
            motorFrameRL = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameRL->SetName("motorFrameRL");
            motorFrameRL->SetPos(ChVector<>(-WB/2.0, TW/2.0-0.15, R_W));
            motorFrameRL->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_Y));
            motorFrameRL->SetBodyFixed(false);
            my_system.AddBody(motorFrameRL);
            
            // --- Motor RR ---
            motorFrameRR = std::make_shared<ChBodyEasyCylinder>(0.09, 0.30, 1600.0, true, true);
            motorFrameRR->SetName("motorFrameRR");
            motorFrameRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0+0.15, R_W));
            motorFrameRR->SetRot(Q_from_AngAxis(-CH_C_PI_2, VECT_Y));
            motorFrameRR->SetBodyFixed(false);
            my_system.AddBody(motorFrameRR);
            
            // --- Wheels ---
            wheelFL = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelFL->SetName("WheelFL");
            wheelFL->SetPos(ChVector<>(WB/2.0, TW/2.0, R_W));
            wheelFL->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_Y));
			wheelFL->SetBodyFixed(false);
            wheelFL->AddAsset(texture);
            my_system.AddBody(wheelFL);
            
            // --- Wheels ---
            wheelFR = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelFR->SetName("WheelFR");
            wheelFR->SetPos(ChVector<>(WB/2.0, -TW/2.0, R_W));
            wheelFR->SetRot(Q_from_AngAxis(-CH_C_PI_2, VECT_Y));
			wheelFR->SetBodyFixed(false);
            wheelFR->AddAsset(texture);
            my_system.AddBody(wheelFR);
            
            // --- Wheels ---
            wheelRL = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelRL->SetName("WheelRL");
            wheelRL->SetPos(ChVector<>(-WB/2.0, TW/2.0, R_W));
            wheelRL->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_Y));
			wheelRL->SetBodyFixed(false);
            wheelRL->AddAsset(texture);
            my_system.AddBody(wheelRL);
            
            // --- Wheels ---
            wheelRR = std::make_shared<ChBodyEasyCylinder>(0.21, 0.20, 720.0, true, true);
            wheelRR->SetName("WheelRR");
            wheelRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0, R_W));
            wheelRR->SetRot(Q_from_AngAxis(-CH_C_PI_2, VECT_Y));
			wheelRR->SetBodyFixed(false);
            wheelRR->AddAsset(texture);
            my_system.AddBody(wheelRR);
            
            // R-joints between wheel and motor frames
            link_revoluteFL = std::make_shared<ChLinkLockRevolute>();
            link_revoluteFL->SetName("WheelAxisFL");
            link_revoluteFL->Initialize(wheelFL, motorFrameFL, \
                             ChCoordsys<>(ChVector<>(WB/2.0, TW/2.0-0.05, R_W), Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
            link_revoluteFL->GetMarker1()->SetName("WAFL");
            link_revoluteFL->GetMarker2()->SetName("MAFL");
            my_system.AddLink(link_revoluteFL);
            
            link_revoluteFR = std::make_shared<ChLinkLockRevolute>();
            link_revoluteFR->SetName("WheelAxisFR");
            link_revoluteFR->Initialize(wheelFR, motorFrameFR, \
                             ChCoordsys<>(ChVector<>(WB/2.0, -TW/2.0+0.05, R_W), Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
            link_revoluteFR->GetMarker1()->SetName("WAFR");
            link_revoluteFR->GetMarker2()->SetName("MAFR");
            my_system.AddLink(link_revoluteFR);
            
            link_revoluteRL = std::make_shared<ChLinkLockRevolute>();
            link_revoluteRL->SetName("WheelAxisRL");
            link_revoluteRL->Initialize(wheelRL, motorFrameRL, \
                             ChCoordsys<>(ChVector<>(-WB/2.0, TW/2.0-0.05, R_W), Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
            link_revoluteRL->GetMarker1()->SetName("WARL");
            link_revoluteRL->GetMarker2()->SetName("MARL");
            my_system.AddLink(link_revoluteRL);
            
            link_revoluteRR = std::make_shared<ChLinkLockRevolute>();
            link_revoluteRR->SetName("WheelAxisRR");
            link_revoluteRR->Initialize(wheelRR, motorFrameRR, \
                             ChCoordsys<>(ChVector<>(-WB/2.0, -TW/2.0+0.05, R_W), Q_from_AngAxis(CH_C_PI_2, VECT_Y)));
            link_revoluteRR->GetMarker1()->SetName("WARR");
            link_revoluteRR->GetMarker2()->SetName("MARR");
            my_system.AddLink(link_revoluteRR);
            
            // Markers for localizing wheel centers
			posFL = std::make_shared<ChMarker>();
			posFL->SetName("SteerAxisFL");
			posFL->SetBody(chassis.get());
			chassis->AddMarker(posFL);
			posFL->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(WB/2.0, TW/2.0, R_W)));//chassis->GetCoord());
			posFL->SetPos(ChVector<>(WB/2.0, TW/2.0, R_W));
			
			posFR = std::make_shared<ChMarker>();
			posFR->SetName("SteerAxisFR");
			posFR->SetBody(chassis.get());
			chassis->AddMarker(posFR);
			posFR->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(WB/2.0, -TW/2.0, R_W)));//chassis->GetCoord());
			posFR->SetPos(ChVector<>(WB/2.0, -TW/2.0, R_W));
			
			posRL = std::make_shared<ChMarker>();
			posRL->SetName("SteerAxisRL");
			posRL->SetBody(chassis.get());
			chassis->AddMarker(posRL);
			posRL->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(-WB/2.0, TW/2.0, R_W)));//chassis->GetCoord());
			posRL->SetPos(ChVector<>(-WB/2.0, TW/2.0, R_W));
			
			posRR = std::make_shared<ChMarker>();
			posRR->SetName("SteerAxisRR");
			posRR->SetBody(chassis.get());
			chassis->AddMarker(posRR);
			posRR->Impose_Abs_Coord(ChCoordsys<>(ChVector<>(-WB/2.0, -TW/2.0, R_W)));//chassis->GetCoord());
			posRR->SetPos(ChVector<>(-WB/2.0, -TW/2.0, R_W));
			
            suspFL = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, true);  // x,y,z,Rx,Ry,Rz constrains, should be (false, false, false, true, true, false)
            suspFL->SetName("SuspensionFL");
            ChFrame<> susp_abs_FL(ChVector<>(WB/2.0, TW/2.0, R_W*2.0+0.3)); //posFL->GetPos()
            suspFL->Initialize(wheelFL,             // the 1st body to connect
                               chassis,             // the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_FL,         // the link reference attached to 1st body
                               susp_abs_FL);        // the link reference attached to 2nd body
            my_system.Add(suspFL);
            
            suspFR = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, true);  // x,y,z,Rx,Ry,Rz constrains
            suspFR->SetName("SuspensionFR");
            ChFrame<> susp_abs_FR(ChVector<>(WB/2.0, -TW/2.0, R_W*2.0+0.3));
            suspFR->Initialize(wheelFR,             // the 1st body to connect
                               chassis,             // the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_FR,         // the link reference attached to 1st body
                               susp_abs_FR);        // the link reference attached to 2nd body
            my_system.Add(suspFR);
            
            suspRL = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, true);  // x,y,z,Rx,Ry,Rz constrains
            suspRL->SetName("SuspensionRL");
            ChFrame<> susp_abs_RL(ChVector<>(-WB/2.0, TW/2.0, R_W*2.0+0.3));
            suspRL->Initialize(wheelRL,           	// the 1st body to connect
                               chassis,        		// the 2nd body to connect
                               false,               // the two following frames are in absolute, not relative, coords.
                               susp_abs_RL,         // the link reference attached to 1st body
                               susp_abs_RL);        // the link reference attached to 2nd body
            my_system.Add(suspRL);
            
            suspRR = std::make_shared<ChLinkMateGeneric>(false, false, false, true, true, true);  // x,y,z,Rx,Ry,Rz constrains
            suspRR->SetName("SuspensionRR");
            ChFrame<> susp_abs_RR(ChVector<>(-WB/2.0, -TW/2.0, R_W*2.0+0.3));
            suspRR->Initialize(wheelRR,           	// the 1st body to connect
                               chassis,        		// the 2nd body to connect
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
    
int sim_physics(int argc, char* argv[]) {
    //
    // HERE YOU CREATE THE MECHANICAL SYSTEM OF CHRONO...
    //
    // 1- Create a ChronoENGINE physical system: all bodies and constraints
    //    will be handled by this ChSystemNSC object.
    SetChronoDataPath(CHRONO_DATA_DIR);
    ChSystemNSC my_system;
    my_system.Set_G_acc(ChVector<>(0., 0., -9.81));
    //ChFrameMoving<> root_frame(ChVector<>(0, 0, 0));
    ChVector<> trackPoint(0.0, 0.0, 1.75);
    // 2.- Create the Irrlicht visualization.
    ChIrrApp application(&my_system, L"Agile Vehicle Simulator", core::dimension2d<u32>(640, 480), false);
    //ChIrrWizard::add_typical_Logo(application.GetDevice());
    
    //ChIrrWizard::add_typical_Sky(application.GetDevice()); //with the following lines:
    std::string mtexturedir = GetChronoDataFile("skybox/");
    std::string str_lf = mtexturedir + "sky_lf.jpg";
    std::string str_up = mtexturedir + "sky_up.jpg";
    std::string str_dn = mtexturedir + "sky_dn.jpg";
    irr::video::ITexture* map_skybox_side = application.GetVideoDriver()->getTexture(str_lf.c_str());
    irr::scene::ISceneNode* mbox = application.GetSceneManager()->addSkyBoxSceneNode(
        application.GetVideoDriver()->getTexture(str_up.c_str()), application.GetVideoDriver()->getTexture(str_dn.c_str()), map_skybox_side,
        map_skybox_side, map_skybox_side, map_skybox_side);
    mbox->setRotation(irr::core::vector3df(90, 0, 0));
    
    ChIrrWizard::add_typical_Lights(application.GetDevice(), irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    //application.SetChaseCamera(trackPoint, 6.0, 0.5);//(TODO)
    
    ChIrrWizard::add_typical_Camera(application.GetDevice());
    application.SetTryRealtime(true);
    
    // 3- Create the rigid bodies of the simpified car suspension mechanical system
    //   maybe setting position/mass/inertias of
    //   their center of mass (COG) etc.
    // ..the world
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("blu.png"));
    
    auto ground = std::make_shared<ChBodyEasyBox>(1000.0, 1000.0, 0.2, 2000.0, true, false);
    ground->SetIdentifier(-1);
    ground->SetName("ground");
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    my_system.AddBody(ground);
    
    for (int i = 0; i < 6; i++) {
        auto my_obstacle = std::make_shared<ChBodyEasyBox>(1, 0.1, 0.5, 60.0, true, true);
        my_obstacle->SetPos(ChVector<>(20 * ChRandom(), 2, 20 * ChRandom()));
        my_obstacle->SetMass(3);
        my_system.AddBody(my_obstacle);
    }
    
    AgileVehicle* AgileV = new AgileVehicle(my_system, application.GetSceneManager(), application.GetVideoDriver());
    utils::ChChaseCamera my_camera(AgileV->chassis);
    my_camera.Initialize(trackPoint, AgileV->chassis->GetCoord(), 6.0, 0.5); //(ptOnChassis, m_vehicle->GetChassis()->GetLocalDriverCoordsys(), chaseDist, chaseHeight)
    my_camera.Update(0.3);
    // Bind visualization assets.
    application.AssetBindAll();
    application.AssetUpdateAll();
    my_system.ShowHierarchy(GetLog());
    
    ChRealtimeStepTimer m_realtime_timer;
    //AgileV->TestSteering(AgileV->motorFrameFL, 1.0);
    //AgileV->TestDriving(AgileV->wheelFR, 1.0);
    while (application.GetDevice()->run() && no_quit) {
        //TODO: When the simulation starts the car flips over, don't know why.
        // Irrlicht must prepare frame to draw
        application.BeginScene(true, true, SColor(255, 140, 161, 192));
        // Irrlicht now draws simple lines in 3D world representing a
        // skeleton of the mechanism, in this instant:
        //
        // .. draw solid 3D items (boxes, cylinders, shapes) belonging to Irrlicht scene, if any
        application.DrawAll();
        // Draw an XY grid at the global origin to add in visualization.
        ChIrrTools::drawGrid(application.GetVideoDriver(), 1, 1, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngZ(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);
        // .. draw GUI user interface items (sliders, buttons) belonging to Irrlicht screen, if any
        application.GetIGUIEnvironment()->drawAll();
        // .. draw the distance constraints (the massless rods) as simplified lines
        ChIrrTools::drawSegment(application.GetVideoDriver(), (AgileV->posFL->GetAbsFrame()).GetPos(), AgileV->motorFrameFL->GetPos(), video::SColor(255, 0, 20, 0), true);
        ChIrrTools::drawSegment(application.GetVideoDriver(), (AgileV->posFR->GetAbsFrame()).GetPos(), AgileV->motorFrameFR->GetPos(), video::SColor(255, 0, 20, 0), true);
        ChIrrTools::drawSegment(application.GetVideoDriver(), (AgileV->posRL->GetAbsFrame()).GetPos(), AgileV->motorFrameRL->GetPos(), video::SColor(255, 0, 20, 0), true);
        ChIrrTools::drawSegment(application.GetVideoDriver(), (AgileV->posRR->GetAbsFrame()).GetPos(), AgileV->motorFrameRR->GetPos(), video::SColor(255, 0, 20, 0), true);
        ChIrrTools::drawSpring(application.GetVideoDriver(), 0.03, (AgileV->posFL->GetAbsFrame()).GetPos(), AgileV->motorFrameFL->GetPos(),video::SColor(255, 150, 20, 20), 80, 5, true);
        ChIrrTools::drawSpring(application.GetVideoDriver(), 0.03, (AgileV->posFR->GetAbsFrame()).GetPos(), AgileV->motorFrameFR->GetPos(),video::SColor(255, 150, 20, 20), 80, 5, true);
        ChIrrTools::drawSpring(application.GetVideoDriver(), 0.03, (AgileV->posRL->GetAbsFrame()).GetPos(), AgileV->motorFrameRL->GetPos(),video::SColor(255, 150, 20, 20), 80, 5, true);
        ChIrrTools::drawSpring(application.GetVideoDriver(), 0.03, (AgileV->posRR->GetAbsFrame()).GetPos(), AgileV->motorFrameRR->GetPos(),video::SColor(255, 150, 20, 20), 80, 5, true);
        //ChIrrTools::drawAllLinks(my_system, application.GetVideoDriver()); //BAD EFFECTS.
        
        
        //TO BE POPULATED.
        
        
            //Add Custom Forces to support the system
            AgileV->setSuspensionForce(AgileV->chassis, AgileV->posFL, AgileV->motorFrameFL, AgileV->wheelFL, 0, &application);
            AgileV->setSuspensionForce(AgileV->chassis, AgileV->posFR, AgileV->motorFrameFR, AgileV->wheelFR, 1, &application);
            AgileV->setSuspensionForce(AgileV->chassis, AgileV->posRL, AgileV->motorFrameRL, AgileV->wheelRL, 2, &application);
            AgileV->setSuspensionForce(AgileV->chassis, AgileV->posRR, AgileV->motorFrameRR, AgileV->wheelRR, 3, &application);
            
            //AgileV->setTyreForce(&my_system, AgileV->chassis, AgileV->wheelFL, 0);
            //AgileV->setTyreForce(&my_system, AgileV->chassis, AgileV->wheelFR, 1);
            //AgileV->setTyreForce(&my_system, AgileV->chassis, AgileV->wheelRL, 2);
            //AgileV->setTyreForce(&my_system, AgileV->chassis, AgileV->wheelRR, 3);
        
        
        // HERE CHRONO INTEGRATION IS PERFORMED: THE
        // TIME OF THE SIMULATION ADVANCES FOR A SINGLE
        // STEP:
        my_system.DoStepDynamics(m_realtime_timer.SuggestSimulationStep(0.002));
        printf("Speed:%f %f %f \n", (AgileV->chassis->GetFrame_COG_to_abs()).GetPos_dt().x(), (AgileV->chassis->GetFrame_COG_to_abs()).GetPos_dt().y(), (AgileV->chassis->GetFrame_COG_to_abs()).GetPos_dt().z());
        printf("Position:%f %f %f \n", (AgileV->chassis->GetFrame_COG_to_abs()).GetPos().x(), (AgileV->chassis->GetFrame_COG_to_abs()).GetPos().y(), (AgileV->chassis->GetFrame_COG_to_abs()).GetPos().z());
        // Irrlicht must finish drawing the frame
        application.EndScene();
    }
    
    if (AgileV)
        delete AgileV;
    
    return 0;
}        
