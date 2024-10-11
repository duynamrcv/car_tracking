import numpy as np
import pyquaternion
import pandas as pd
import matplotlib.pyplot as plt

def quaternion_to_euler(q):
    q = pyquaternion.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    yaw, pitch, roll = q.yaw_pitch_roll
    return [roll, pitch, yaw]

def load_csv(file_name):
    df = pd.read_csv(file_name, sep=' ')
    df = np.array(df)
    
    timestamp = df[:,0]
    x = df[:,1]
    y = df[:,2]
    # z = df[:,3]

    theta = []
    for i in range(df.shape[0]):
        _, _, yaw = quaternion_to_euler(df[i,4:8])
        theta.append(yaw)
    gear = df[:,8]
    return timestamp, x, y, theta, gear

if __name__ == "__main__":
    # file_name = "basement1.txt"
    # timestamp, x, y, theta, gear = load_csv(file_name)

    file_name = "trajectory.txt"
    df = pd.read_csv(file_name, sep=',')
    df = np.array(df)
    
    x = df[:,0]
    y = df[:,1]

    bx = [0.0, -3.5, 2.0, 10.0, 3.0, 5.0, -2.0]
    by = [0.0, 2.0, -1.5, 5.0, 5.0, 10.0, 3.0]

    plt.figure()
    plt.plot(bx, by, "xb")
    plt.plot(x, y, "-or")
    plt.show()