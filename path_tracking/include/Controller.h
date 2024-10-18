#pragma once

// standard
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_CarModel.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#define NX   CARMODEL_NX
#define NP   CARMODEL_NP
#define NU   CARMODEL_NU
#define NY   CARMODEL_NY
#define NBX0 CARMODEL_NBX0
#define N    CARMODEL_N
#define NBU  CARMODEL_NBU

struct WayPoint
{
    double x;
    double y;
    double yaw;
    double v;
    double steer;
};

struct ControlSignal
{
    double speed;
    double steering;
    double accel;
    int gear;  // -1, 0, 1: backward, neutral, forward
};

class Controller
{
public:
    Controller();
    ~Controller();

    void setContraints(double maxV, double minV, double maxSteering, double minSteering);
    void setWeights(const double weight[NY]);
    void setParmeters(double wheelbase);

    int solve(const double currentState[3], const std::vector<WayPoint> localTrajectory,
              ControlSignal &signal);

private:
    CarModel_solver_capsule *acadosOcpCapsule_;
    ocp_nlp_config *nlpConfig_;
    ocp_nlp_dims *nlpDims_;
    ocp_nlp_in *nlpIn_;
    ocp_nlp_out *nlpOut_;
    ocp_nlp_solver *nlpSolver_;
    void *nlpOpts_;

    double parameter[NP];  // Parameter
    double *W_, *W_e_;     // Weight
    double *lbu, *ubu;     // Constraint
};