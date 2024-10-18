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

struct WayPoints
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

    int solve(const double currentState[3], const std::vector<WayPoints> localTrajectory,
              ControlSignal &signal);

private:
    CarModel_solver_capsule *acados_ocp_capsule;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;

    double parameter[NP];  // Parameter
    double *W_, *W_e_;     // Weight
    double *lbu, *ubu;     // Constraint
};