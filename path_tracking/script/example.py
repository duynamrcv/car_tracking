import math

from car_model import Car
from controller_opt import Controller

import numpy as np
import timeit
import pandas as pd


def load_trajectory(file_name):
    df = pd.read_csv(file_name, sep='\t')
    # print(df.head())
    x_ref = np.array(df['x'])
    y_ref = np.array(df['y'])
    psi_ref = np.array(df['theta'])
    return x_ref, y_ref, psi_ref


def load_path(file):
    df = pd.read_csv(file)
    # print(df.head())
    x_ref = np.array(df['x'])
    y_ref = np.array(df['y'])
    psi_ref = np.array(df['theta'])
    dirs_ref = np.array(df['dir'])

    return x_ref, y_ref, psi_ref, dirs_ref


def gen_2sub_path(x_ref, y_ref, psi_ref, dirs_ref):
    # create 2 sub paths
    x_ref1 = []
    y_ref1 = []
    psi_ref1 = []
    dirs_ref1 = []
    x_ref2 = []
    y_ref2 = []
    psi_ref2 = []
    dirs_ref2 = []
    for i in range(len(x_ref)):
        if dirs_ref[i] == 1:
            x_ref1.append(x_ref[i])
            y_ref1.append(y_ref[i])
            psi_ref1.append(psi_ref[i])
            dirs_ref1.append(dirs_ref[i])
        else:
            x_ref2.append(x_ref[i])
            y_ref2.append(y_ref[i])
            psi_ref2.append(psi_ref[i])
            dirs_ref2.append(dirs_ref[i])

    # add 10 points to the end of the path
    for i in range(100):
        x_ref1.append(x_ref1[-1])
        y_ref1.append(y_ref1[-1])
        psi_ref1.append(psi_ref1[-1])
        dirs_ref1.append(dirs_ref1[-1])

    return x_ref1, y_ref1, psi_ref1, dirs_ref1, x_ref2, y_ref2, psi_ref2, dirs_ref2


N = 20  # number of discretization steps
T = 20.00  # maximum simulation time[s]
dt = 0.1  # time step[s]

file_name = "/home/duynam/car_tracking/data/reference.txt"
xref_pre, yref_pre, theta_ref_pre, dirs_pre = load_path(file_name)

xref, yref, theta_ref, dirsref, xref2, yref2, theta_ref2, dirsref2 = gen_2sub_path(xref_pre, yref_pre, theta_ref_pre, dirs_pre)
#xref1, yref1, theta_ref1, dirsref1, xref, yref, theta_ref, dirsref = gen_2sub_path(xref_pre, yref_pre, theta_ref_pre, dirs_pre)


model_file = "/home/duynam/car_tracking/path_tracking/config/car_model.yaml"
car = Car(np.array([xref[0], yref[0], theta_ref[0]]), model_file)
controller = Controller(car, t_horizon=N*dt, n_nodes=N,
                        q_cost=np.array([10., 10., 0.01]), r_cost=np.array([0.1, 0.1])
                        )

wheelbase = car.wheelbase
p_value = np.array([car.wheelbase])

time_record = []
control_record = []
current_time = 0
for i in range(len(xref)):
    x_current = car.state

    for j in range(N):
        index = i + j
        if index >= len(xref):
            index = -1
        y_ref = np.array([xref[index], yref[index], theta_ref[index], 0, 0])
        controller.acados_ocp_solver.set(j, 'yref', y_ref)
        controller.acados_ocp_solver.set(j, 'p', np.array([2.90909]))

    indexN = i + controller.N
    if indexN >= len(xref):
        indexN = -1
    y_refN = np.array([xref[indexN], yref[indexN], theta_ref[indexN]])
    controller.acados_ocp_solver.set(N, 'yref', y_refN)
    controller.acados_ocp_solver.set(N, 'p', p_value)

    # solve ocp
    start = timeit.default_timer()
    controller.acados_ocp_solver.set(0, 'lbx', x_current)
    controller.acados_ocp_solver.set(0, 'ubx', x_current)
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
ref_record = np.array([xref, yref, theta_ref]).T

print("average estimation time is {:.4f} ms".format(time_record.mean()*10e3))
print("max estimation time is {:.4f} ms".format(time_record.max()*10e3))
print("min estimation time is {:.4f} ms".format(time_record.min()*10e3))

# plot steer and velocity
import matplotlib.pyplot as plt

plt.figure()
plt.plot(path_record[:,0], control_record[:,0])
plt.plot(path_record[:,0], control_record[:,1])
plt.legend(['steer', 'velocity'])
plt.grid(True)

plt.figure()
plt.plot(ref_record[:,0], ref_record[:,1])
plt.plot(path_record[:,1], path_record[:,2])
plt.grid(True)

plt.show()
