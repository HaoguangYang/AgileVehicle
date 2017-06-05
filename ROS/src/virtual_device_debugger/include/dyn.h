#ifndef DYN_H
#define DYN_H
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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool getTireForces(double load, double omega, double def_z, double roll_r, double v_wx, double v_wy, double v_wz, \
				   double F_vert, double F_lat, double F_long, double T_ali);

int sim_physics(int argc, char* argv[]);

#endif
