#ifndef DYN_H
#define DYN_H
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/core/ChTimer.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChFrame.h"
#include "chrono/physics/ChMarker.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include <limits>
#include <cstdio>
#include <cmath>
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
using namespace chrono::vehicle;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

extern bool no_quit;

bool getTireForces(double load, double omega, double def_z, double roll_r, double v_wx, double v_wy, double v_wz, \
				   double F_vert, double F_lat, double F_long, double T_ali);

int sim_physics(int argc, char* argv[]);

#endif
