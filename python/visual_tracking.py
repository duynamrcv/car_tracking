import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

ref_file = "data/reference.txt"
df = pd.read_csv(ref_file, sep=',')
df = np.array(df)

xr = df[:,0]
yr = df[:,1]
yawr = df[:,2]

rb_file = "data/motion_path.csv"
df = pd.read_csv(rb_file, sep=',')
df = np.array(df)

x = df[:,0]
y = df[:,1]
yaw = df[:,2]

plt.figure()
plt.plot(xr, yr)
plt.plot(x, y)
plt.grid(True)

plt.show()
