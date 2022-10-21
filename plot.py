import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

fileDir = "trained.csv" #"out.csv"

data = pd.read_csv(fileDir, engine = 'c', float_precision = 'round_trip', dtype=np.float64)

fig = plt.figure(figsize=(11.5, 8.5))
ax = fig.add_subplot(111, projection='3d')

ax.plot_trisurf(data['x'], data['y'], data['z'], cmap='viridis', alpha=0.5)

plt.show()