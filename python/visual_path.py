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
    file_name = "data/basement1.txt"
    timestamp, bx, by, theta, gear = load_csv(file_name)

    file_name = "data/trajectory0.csv"
    df = pd.read_csv(file_name, sep=',')
    df = np.array(df)
    
    x = df[:,0]
    y = df[:,1]

    file_name = "data/trajectory1.csv"
    df = pd.read_csv(file_name, sep=',')
    df = np.array(df)
    
    cx = df[:,0]
    cy = df[:,1]

    plt.figure()
    plt.plot(bx, by, "xb", label="Sub-path 1")
    plt.plot(x, y, "-or", label="Sub-path 2")
    plt.plot(cx, cy, "-og", label="Cubic Spline")
    plt.axis('scaled')
    plt.legend()
    plt.show()