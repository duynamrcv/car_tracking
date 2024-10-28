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
        self.max_a = config['max_acceleration']
        self.max_steer = np.deg2rad(config['max_steering'])
        self.max_delta = np.deg2rad(config['max_rating'])
        
        self.min_v = -self.max_v
        self.min_a = -self.max_a
        self.min_steer = -self.max_steer
        self.min_delta = -self.max_delta

        # Initial path
        self.stamp = 0
        self.path = []

    def f_dynamic(self, state:np.array, control:np.array):
        return np.array([
            state[3] * np.cos(state[2]),
            state[3] * np.sin(state[2]),
            state[3] * np.tan(state[4]) / self.wheelbase,
            control[0],
            control[1]
        ])

    def update_state(self, control, dt):
        # Using Euler
        p = self.state
        self.state = p + self.f_dynamic(p, control) * dt

        self.stamp += dt
        self.path.append(np.concatenate([[self.stamp], self.state]))
