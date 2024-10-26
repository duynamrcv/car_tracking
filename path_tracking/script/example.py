import math

from car_model import Car
from controller_opt import Controller

import numpy as np
import timeit
import pandas as pd


def load_trajectory(file_name):
    df = pd.read_csv(file_name, sep=',')
    ref = np.array(df)[:,:3]
    return ref

def normal_angle(theta):
    theta -= np.floor((theta + np.pi)/(2*np.pi))*2*np.pi
    return theta

def get_local_reference(reference, state, index, num_point):
    local_reference = []
    for i in range(num_point):
        idx = index + i
        if idx >= reference.shape[0]:
            idx = reference.shape[0] - 1

        # Convert global to local
        dx = reference[idx,0] - state[0]
        dy = reference[idx,1] - state[1]
        x  = dx * np.cos(-state[2]) - dy * np.sin(-state[2])
        y  = dx * np.sin(-state[2]) + dy * np.cos(-state[2])
        yaw = normal_angle(reference[idx,2] - state[2])
        local_reference.append(np.array([x, y, yaw]))
    return np.array(local_reference)

N = 30  # number of discretization steps
T = 20.00  # maximum simulation time[s]
dt = 0.1  # time step[s]

file_name = "/home/nambd3/spline_path/data/reference.txt"
ref = load_trajectory(file_name)

model_file = "/home/nambd3/spline_path/path_tracking/config/car_model.yaml"
car = Car(np.array([ref[0,0], ref[0,1], ref[0,2], 0, 0]), model_file)
controller = Controller(car, t_horizon=N*dt, n_nodes=N,
                        q_cost=np.array([10., 10., 0.01, 1., 1.]), r_cost=np.array([0.1, 0.1])
                        )

wheelbase = car.wheelbase
p_value = np.array([car.wheelbase])

time_record = []
control_record = []
current_time = 0
for i in range(ref.shape[0]):
    x_current = car.state
    x_local = np.array([0,0,0,x_current[3],x_current[4]])
    local_ref = get_local_reference(ref, x_current, i, N+1)

    for j in range(N):
        y_ref = np.array([local_ref[j,0], local_ref[j,1], local_ref[j,2], 0, 0, 0, 0])
        controller.acados_ocp_solver.set(j, 'yref', y_ref)
        controller.acados_ocp_solver.set(j, 'p', np.array([2.95]))

    y_refN = np.array([local_ref[N,0], local_ref[N,1], local_ref[N,2], 0, 0])
    controller.acados_ocp_solver.set(N, 'yref', y_refN)
    controller.acados_ocp_solver.set(N, 'p', p_value)

    # solve ocp
    start = timeit.default_timer()
    controller.acados_ocp_solver.set(0, 'lbx', x_local)
    controller.acados_ocp_solver.set(0, 'ubx', x_local)
    status = controller.acados_ocp_solver.solve()

    if status != 0:
        raise Exception("acados_ocp_solver returned status {}. Exiting.".format(status))

    u = controller.acados_ocp_solver.get(0, 'u')
    time_record.append(timeit.default_timer() - start)

    # simulate system
    car.update_state(u, dt)

    # save control signal
    control_record.append(u)
    current_time += dt

time_record = np.array(time_record)
control_record = np.array(control_record)
path_record = np.array(car.path)

print("average estimation time is {:.4f} ms".format(time_record.mean()*10e3))
print("max estimation time is {:.4f} ms".format(time_record.max()*10e3))
print("min estimation time is {:.4f} ms".format(time_record.min()*10e3))

# plot steer and velocity
import matplotlib.pyplot as plt

plt.figure()
plt.plot(path_record[:,0], path_record[:,4])
plt.plot(path_record[:,0], path_record[:,5])
plt.plot(path_record[:,0], control_record[:,0])
plt.plot(path_record[:,0], control_record[:,1])
plt.legend(['velocity', 'steer', 'acceleration', 'steer_rate'])
plt.grid(True)

plt.figure()
plt.plot(ref[:,0], ref[:,1])
plt.plot(path_record[:,1], path_record[:,2])
plt.grid(True)

plt.show()
