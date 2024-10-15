#!/usr/bin/env python3
# coding=UTF-8

import os

import numpy as np
import casadi as ca
import scipy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from car_model import Car

class Controller:
    def __init__(self, car:Car, t_horizon=1.0, n_nodes=20, model_name="CarModel",
                 q_cost=None, r_cost=None):
        self.T = t_horizon
        self.N = n_nodes

        self.car = car

        self.acados_model = self.acados_setup_model(model_name)
        self.acados_ocp_solver = self.acados_setup_controller(q_cost, r_cost)

    def acados_setup_model(self, model_name):
        # Control inputs
        v = ca.SX.sym('v')
        steer = ca.SX.sym('delta')
        controls = ca.vertcat(v, steer)

        # States
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        yaw = ca.SX.sym('yaw')
        states = ca.vertcat(x, y, yaw)
        
        # Acados model
        states_dot = ca.SX.sym("states_dot", states.shape)

        # function
        rhs = [v * ca.cos(yaw), v * ca.sin(yaw), v * ca.tan(steer) / self.car.wheelbase]
        f_expl = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])
        f_impl = states_dot - f_expl(states, controls)

        # model configuration
        model = AcadosModel()  # ca.types.SimpleNamespace()
        model.f_expl_expr = f_expl(states, controls)  # CasADi expression for the explicit dynamics x˙=fexpl(x,u,p).
        # Used if acados_template.acados_ocp.AcadosOcpOptions.integrator_type == ‘ERK.’
        model.f_impl_expr = f_impl  # CasADi expression for the implicit dynamics fimpl(x˙,x,u,z,p)=0.
        # Used if acados_template.acados_ocp.AcadosOcpOptions.integrator_type == ‘IRK.’
        model.x = states  # CasADi variable describing the state of the system
        model.xdot = states_dot  # CasADi variable describing the time-derivative of the state
        model.u = controls  # CasADi variable describing the control input of the system
        model.p = []  # CasADi variable describing the parameters of the system (not used here)
        model.name = model_name  # Name of the system
        
        return model
    
    def acados_setup_controller(self, q_cost, r_cost):
        nx = self.acados_model.x.size()[0]  # number of states [x, y, yaw]
        nu = self.acados_model.u.size()[0]  # number of control inputs [v, steer]
        ny = nx + nu  # number of variables in total [x, y, yaw, v, steer] for each node
        n_params = len(self.acados_model.p)  # number of parameters to be optimized

        ocp = AcadosOcp()
        ocp.model = self.acados_model
        ocp.dims.N = self.N  
        ocp.solver_options.tf = self.T

        # Initialize parameters
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)

        # Cost type
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        ocp.cost.W = scipy.linalg.block_diag(q_cost, r_cost)  # weight matrix for states and control inputs
        ocp.cost.W_e = q_cost  # weight matrix for the final state

        ocp.cost.Vx = np.zeros((ny, nx))  # x matrix coefficient at intermediate shooting nodes (1 to N-1).
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)  # identity matrix

        ocp.cost.Vu = np.zeros((ny, nu))  # u matrix coefficient at intermediate shooting nodes (1 to N-1).
        ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        ocp.cost.Vx_e = np.eye(nx)

        # Set initial condition
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(nx)

        # Set constraints
        ocp.constraints.x0 = np.zeros(nx)
        ocp.constraints.lbu = np.array([self.car.min_v, self.car.min_steer])
        ocp.constraints.ubu = np.array([self.car.max_v, self.car.max_steer])
        ocp.constraints.idxbu = np.array([0, 1])

        # Set QP solver and integration
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        # Compile acados OCP solver if necessary
        json_file = os.path.join('./' + self.acados_model.name + '_acados_ocp.json')
        solver = AcadosOcpSolver(ocp, json_file=json_file)
        
        return solver