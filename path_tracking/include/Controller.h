#pragma once

// standard
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "CarModel.h"

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
#define NBX  CARMODEL_NBX

class Controller
{
public:
    Controller();
    ~Controller();

    void setContraints(double maxV, double minV, double maxSteering, double minSteering,
                       double maxA, double minA, double maxSteerRate, double minSteerRate);
    void setWeights(const double weight[NY]);
    void setParmeters(double wheelbase);
    void setTimeStep(double timeStep);

    int solve(const Pose &currentPose, const std::vector<Pose> localTrajectory,
              ControlSignal &signal);

private:
    CarModel_solver_capsule *acadosOcpCapsule_;
    ocp_nlp_config *nlpConfig_;
    ocp_nlp_dims *nlpDims_;
    ocp_nlp_in *nlpIn_;
    ocp_nlp_out *nlpOut_;
    ocp_nlp_solver *nlpSolver_;
    void *nlpOpts_;

    double parameter[NP];               // Parameter
    double W_[NY * NY], W_e_[NX * NX];  // Weight
    double lbx_[NBX], ubx_[NBX];        // Constraint
    double lbu_[NBU], ubu_[NBU];        // Constraint
    double timeStep_;                   // Time step
};