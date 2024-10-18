#include "Controller.h"
#include <iostream>
#include <cmath>

Controller::Controller()
{
    acados_ocp_capsule = CarModel_acados_create_capsule();

    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = CarModel_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        std::cout << "CarModel_acados_create() returned status" << status << ". Exiting.\n";
        exit(1);
    }

    nlp_config = CarModel_acados_get_nlp_config(acados_ocp_capsule);
    nlp_dims   = CarModel_acados_get_nlp_dims(acados_ocp_capsule);
    nlp_in     = CarModel_acados_get_nlp_in(acados_ocp_capsule);
    nlp_out    = CarModel_acados_get_nlp_out(acados_ocp_capsule);
    nlp_solver = CarModel_acados_get_nlp_solver(acados_ocp_capsule);
    nlp_opts   = CarModel_acados_get_nlp_opts(acados_ocp_capsule);

    W_   = (double*)calloc(NY * NY, sizeof(double));
    W_e_ = (double*)calloc(NX * NX, sizeof(double));

    double* lubu = (double*)calloc(2 * NBU, sizeof(double));
    lbu          = lubu;
    ubu          = lubu + NBU;

    // Fix parameter, edit to input config
    double wheelbase   = 2.95;
    double weight[NY]  = {100., 100., 0.01, 0.1, 0.1};
    double maxV        = 1.5;
    double minV        = 0.0;
    double maxSteering = M_PI / 6;
    double minSteering = -M_PI / 6;

    setParmeters(wheelbase);
    setWeights(weight);
    setContraints(maxV, minV, maxSteering, minSteering);
}

Controller::~Controller()
{
    // free solver
    int status = CarModel_acados_free(acados_ocp_capsule);
    if (status)
    {
        std::cout << "CarModel_acados_free() returned status " << status << ".\n";
    }
    // free solver capsule
    status = CarModel_acados_free_capsule(acados_ocp_capsule);
    if (status)
    {
        std::cout << "CarModel_acados_free_capsule() returned status " << status << ".\n";
    }
}

void Controller::setParmeters(double wheelbase)
{
    parameter[0] = wheelbase;
}

void Controller::setContraints(double maxV, double minV, double maxSteering, double minSteering)
{
    lbu[0] = minV;
    ubu[0] = maxV;
    lbu[1] = minSteering;
    ubu[1] = maxSteering;
}

void Controller::setWeights(const double weight[NY])
{
    // Set weight for W
    W_[0 + (NY)*0] = weight[0];
    W_[1 + (NY)*1] = weight[1];
    W_[2 + (NY)*2] = weight[2];
    W_[3 + (NY)*3] = weight[3];
    W_[4 + (NY)*4] = weight[4];

    // Set weight for W_e
    W_e_[0 + (NX)*0] = weight[0];
    W_e_[1 + (NX)*1] = weight[1];
    W_e_[2 + (NX)*2] = weight[2];
}

int Controller::solve(const double currentState[3], const std::vector<WayPoints> localTrajectory,
                      ControlSignal& signal)
{
    // check if the number of way points is different from the number of shooting intervals
    if (localTrajectory.size() != N)
    {
        std::cout << "The number of way points is different N = " << N << ". Exiting.\n";
        exit(1);
    }

    for (size_t i = 0; i <= N; i++)
    {
        // set target states and costs
        if (i < N)
        {
            double state[NX + NU];
            state[0] = localTrajectory[i].x;
            state[1] = localTrajectory[i].y;
            state[2] = localTrajectory[i].yaw;
            state[3] = localTrajectory[i].v;
            state[4] = localTrajectory[i].steer;
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", state);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W_);
        }
        else
        {
            double stateN[NX];
            stateN[0] = localTrajectory[N - 1].x;
            stateN[1] = localTrajectory[N - 1].y;
            stateN[2] = localTrajectory[N - 1].yaw;
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", stateN);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e_);
        }

        // set parameters
        CarModel_acados_update_params(acados_ocp_capsule, i, parameter, NP);
    }

    // set initial condition
    double initState[NX];
    for (int i = 0; i < NX; i++)
    {
        initState[i] = currentState[i];
    }
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", initState);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", initState);

    int status_solver = CarModel_acados_solve(acados_ocp_capsule);
    if (status_solver != ACADOS_SUCCESS)
    {
        std::cout << "acados returned status " << status_solver << ". Exiting.\n";
    }

    // get solution
    double u[NU];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u);

    signal.speed    = u[0];
    signal.steering = u[1];
    signal.accel    = 0.0;
    signal.gear     = 1;
    return status_solver;
}