import numpy as np
import pyquaternion
import pandas as pd
import matplotlib.pyplot as plt

file_name = "data/trajectory0.csv"
df = pd.read_csv(file_name, sep=',')
df = np.array(df)

rx = df[1:,0]
ry = df[1:,1]

# rb_file = "data/motion_path.csv"
# df = pd.read_csv(rb_file, sep=',')
# df = np.array(df)

# vehicle_x = df[:,0]
# vehicle_y = df[:,1]

file_name = "data/motion.csv"
df = pd.read_csv(file_name, sep=' ')
df = np.array(df)

# print(vehicle_x.shape)
# print(df.shape)
# exit()

## Plot motion paths
plt.figure(figsize=(8,8))
ax = plt.axes()
for iter in range(df.shape[0]):
    ax.cla()

    # Reference
    ax.plot(rx, ry, "bo", label="Reference")

    # Current vehicle
    # ax.scatter(vehicle_x[iter], vehicle_y[iter], s=100, label="Vehicle")

    # Current motion
    motion = df[iter,:-1].reshape(df[iter,:-1].shape[0]//3, 3)
    ax.plot(motion[:,0], motion[:,1], "-rh", label="Motion path")


    ax.axis('scaled')
    ax.grid(True)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.legend()

    # find center
    # plt.xlim([vehicle_x[iter]-5, vehicle_x[iter]+5])
    # plt.ylim([vehicle_y[iter]-5, vehicle_y[iter]+5])

    plt.xlim([motion[0,0]-5, motion[0,0]+5])
    plt.ylim([motion[0,1]-5, motion[0,1]+5])
    plt.tight_layout()
    
    plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
    plt.pause(0.01)

plt.show()