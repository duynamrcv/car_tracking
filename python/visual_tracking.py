import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

ref_file = "data/trajectory0.csv"
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
v = df[:,3]
steer = df[:,4]

plt.figure(figsize=(4,6))
plt.plot(xr, yr, label="reference")
plt.plot(x, y, label="motion path")
plt.grid(True)
plt.axis('scaled')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.tight_layout()

file_name = "data/tracking.png"
plt.savefig(file_name)

plt.figure()
plt.plot(v)
plt.plot(steer)
plt.grid(True)
plt.show()
