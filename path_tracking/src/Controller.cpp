#include "Controller.h"
#include <cmath>
#include <iostream>

Controller::Controller()
{
    acadosOcpCapsule_ = CarModel_acados_create_capsule();

    // allocate the array and fill it accordingly
    double* newTimeSteps = NULL;
    int status = CarModel_acados_create_with_discretization(acadosOcpCapsule_, N, newTimeSteps);

    if (status)
    {
        std::cout << "CarModel_acados_create() returned status" << status << ". Exiting.\n";
        exit(1);
    }

    nlpConfig_ = CarModel_acados_get_nlp_config(acadosOcpCapsule_);
    nlpDims_   = CarModel_acados_get_nlp_dims(acadosOcpCapsule_);
    nlpIn_     = CarModel_acados_get_nlp_in(acadosOcpCapsule_);
    nlpOut_    = CarModel_acados_get_nlp_out(acadosOcpCapsule_);
    nlpSolver_ = CarModel_acados_get_nlp_solver(acadosOcpCapsule_);
    nlpOpts_   = CarModel_acados_get_nlp_opts(acadosOcpCapsule_);

    // Fix parameter, edit to input config
    double wheelbase   = 2.95;
    double weight[NY]  = {100., 100., 0.01, 0.1, 0.1};
    double maxV        = 1.2;
    double minV        = -1.2;
    double maxSteering = M_PI / 4;
    double minSteering = -M_PI / 4;
    double timeStep    = 0.1;

    setTimeStep(timeStep);
    setParmeters(wheelbase);
    setWeights(weight);
    setContraints(maxV, minV, maxSteering, minSteering);
}

Controller::~Controller()
{
    // free solver
    int status = CarModel_acados_free(acadosOcpCapsule_);
    if (status)
    {
        std::cout << "CarModel_acados_free() returned status " << status << ".\n";
    }
    // free solver capsule
    status = CarModel_acados_free_capsule(acadosOcpCapsule_);
    if (status)
    {
        std::cout << "CarModel_acados_free_capsule() returned status " << status << ".\n";
    }
}

void Controller::setParmeters(double wheelbase)
{
    parameter[0] = wheelbase;
    for (size_t i = 0; i <= N; i++)
    {
        CarModel_acados_update_params(acadosOcpCapsule_, i, parameter, NP);
    }
}

void Controller::setContraints(double maxV, double minV, double maxSteering, double minSteering)
{
    lbu_[0] = minV;
    ubu_[0] = maxV;
    lbu_[1] = minSteering;
    ubu_[1] = maxSteering;
    for (size_t i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlpConfig_, nlpDims_, nlpIn_, i, "lbu", lbu_);
        ocp_nlp_constraints_model_set(nlpConfig_, nlpDims_, nlpIn_, i, "ubu", ubu_);
    }
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

    for (size_t i = 0; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlpConfig_, nlpDims_, nlpIn_, i, "W", W_);
    }
    ocp_nlp_cost_model_set(nlpConfig_, nlpDims_, nlpIn_, N, "W", W_e_);
}

void Controller::setTimeStep(double timeStep)
{
    timeStep_ = timeStep;
    for (size_t i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlpConfig_, nlpDims_, nlpIn_, i, "Ts", &timeStep_);
    }
}

int Controller::solve(const Pose& currentPose, const std::vector<WayPoint> localTrajectory,
                      ControlSignal& signal)
{
    // check if the number of way points is different from the number of shooting intervals
    if (localTrajectory.size() != N)
    {
        std::cout << "The number of way points is different N = " << N << ". Exiting.\n";
        exit(1);
    }

    // set target states
    for (size_t i = 0; i < N; i++)
    {
        double state[NX + NU];
        state[0] = localTrajectory[i].x;
        state[1] = localTrajectory[i].y;
        state[2] = localTrajectory[i].yaw;
        state[3] = localTrajectory[i].v;
        state[4] = localTrajectory[i].steer;
        ocp_nlp_cost_model_set(nlpConfig_, nlpDims_, nlpIn_, i, "yref", state);
    }
    double stateN[NX];
    stateN[0] = localTrajectory[N - 1].x;
    stateN[1] = localTrajectory[N - 1].y;
    stateN[2] = localTrajectory[N - 1].yaw;
    ocp_nlp_cost_model_set(nlpConfig_, nlpDims_, nlpIn_, N, "yref", stateN);

    // set initial condition
    double initState[NX] = {currentPose.x, currentPose.y, currentPose.yaw};
    ocp_nlp_constraints_model_set(nlpConfig_, nlpDims_, nlpIn_, 0, "lbx", initState);
    ocp_nlp_constraints_model_set(nlpConfig_, nlpDims_, nlpIn_, 0, "ubx", initState);

    int status_solver = CarModel_acados_solve(acadosOcpCapsule_);
    if (status_solver != ACADOS_SUCCESS)
    {
        std::cout << "acados returned status " << status_solver << ". Exiting.\n";
    }

    // get solution
    double u[NU];
    ocp_nlp_out_get(nlpConfig_, nlpDims_, nlpOut_, 0, "u", &u);

    signal.speed    = u[0];
    signal.steering = u[1];
    signal.accel    = 0.0;
    signal.gear     = 1;
    return status_solver;
}