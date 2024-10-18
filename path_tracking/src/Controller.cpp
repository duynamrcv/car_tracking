#include "Controller.h"
#include <iostream>

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

    setParmeters();
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

void Controller::setParmeters()
{
    wheelbase_ = 2.95;
}

void Controller::setContraints() {}
void Controller::setWeights() {}

int Controller::solve(const double currentState[3], const std::vector<WayPoints> localTrajectory,
                      ControlSignal& signal)
{
    // check if the number of way points is different from the number of shooting intervals
    if (localTrajectory.size() != N)
    {
        std::cout << "The number of way points is different N = " << N << ". Exiting.\n";
        exit(1);
    }

    // set target states and parameters for each step
    for (size_t i = 0; i < N; i++)
    {
        double state[NX + NU];
        state[0] = localTrajectory[i].x;
        state[1] = localTrajectory[i].y;
        state[2] = localTrajectory[i].yaw;
        state[3] = localTrajectory[i].v;
        state[4] = localTrajectory[i].steer;
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", state);

        double parameter[NP] = {wheelbase_};
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "p", parameter);
    }

    double stateN[NX];
    stateN[0] = localTrajectory[N - 1].x;
    stateN[1] = localTrajectory[N - 1].y;
    stateN[2] = localTrajectory[N - 1].yaw;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", stateN);

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