#!/usr/bin/env python3
# coding=UTF-8

import numpy as np
import yaml
import casadi as ca
from acados_template import AcadosModel

class Car:
    def __init__(self, state:np.array, config_file="config.yaml"):
        # Load config file
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        # The state of car
        self.state = state

        # Define the constraints
        self.wheelbase = config['wheel_base']
        self.max_v = config['max_velocity']
        self.min_v = -self.max_v
        self.max_steer = np.deg2rad(config['max_steering'])
        self.min_steer = -self.max_steer

        # Initial path
        self.stamp = 0
        self.path = []

    def f_dynamic(self, state:np.array, control:np.array):
        return np.array([
            control[0] * np.cos(state[2]),
            control[0] * np.sin(state[2]),
            control[0] * np.tan(control[1]) / self.wheelbase
        ])

    def update_state(self, control, dt):
        method = 1
        if method == 0:
            # Using Runge-Kutta 4
            p = self.state
            k1 = self.f_dynamic(p, control)
            p_aux = p + k1 * dt / 2
            k2 = self.f_dynamic(p_aux, control)
            p_aux = p + k2 * dt / 2
            k3 = self.f_dynamic(p_aux, control)
            p_aux = p + k3 * dt
            k4 = self.f_dynamic(p_aux, control)

            self.state = p + (k1 + 2 * k2 + 2 * k3 + k4 ) / 6.0 * dt
        else:
            # Using Euler
            p = self.state
            self.state = p + self.f_dynamic(p, control) * dt

        self.stamp += dt
        self.path.append(np.concatenate([[self.stamp], self.state]))
