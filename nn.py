import pandas as pd
import numpy as np
import csv
import math
import random

# Make numpy values easier to read.
np.set_printoptions(precision=3, suppress=True)

import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras import regularizers

kick_chance_train = pd.read_csv(
    "out.csv",
    names=["ballX", "ballY", "ballAngle", "ballSpeed",
          "robot0X", "robot0Y", "robot0XVel", "robot0YVel",
          "robot1X", "robot1Y", "robot1XVel", "robot1YVel",
          "x", "y", "z"])

kick_chance_features = kick_chance_train.copy()
kick_chance_labels = kick_chance_features.pop('z')
kick_chance_features.pop("x")
kick_chance_features.pop("y")
kick_chance_features.pop("ballAngle")
kick_chance_features.pop("ballSpeed")
kick_chance_features.pop("robot0X")
kick_chance_features.pop("robot0Y")
kick_chance_features.pop("robot0XVel")
kick_chance_features.pop("robot0YVel")
kick_chance_features.pop("robot1X")
kick_chance_features.pop("robot1Y")
kick_chance_features.pop("robot1XVel")
kick_chance_features.pop("robot1YVel")
kick_chance_features = np.array(kick_chance_features)

normalize = layers.Normalization()
normalize.adapt(kick_chance_features)
# norm_kick_chance_model = tf.keras.Sequential([
#   normalize,
#   #layers.Input(shape=(14)),
#   layers.Dense(128, activation='tanh'),
#   layers.Dense(64, activation='tanh'),
#   layers.Dense(1, activation='sigmoid')
# ]) 2.8336e-4 for just XY input

norm_kick_chance_model = tf.keras.Sequential([
  normalize,
  #layers.Input(shape=(14)),
  layers.Dense(128, activation='tanh'),
  layers.Dense(64, activation='tanh'),
  layers.Dense(1)
]) # 4.6159e-4 for just XY input


norm_kick_chance_model.compile(loss = tf.keras.losses.MeanSquaredError(), metrics=['accuracy'])

norm_kick_chance_model.fit(kick_chance_features, kick_chance_labels, epochs=1000, batch_size=1000, validation_split=.2, shuffle=True)


# X: -3 to 1.4
# Y: -3 to 3
# Keep everything else the same
# Angle to 1.5, 0
#X,Y,ANGLE,3,0,0,0,-1,0,1,0,0.1,X,Y,OUTPUT Z
print(0.311481)
#print(norm_kick_chance_model(np.array([np.array([-3,-3,0.588003,3,0,0,0,-0.1,0,1,0,0.1,-3,-3])])))
print(norm_kick_chance_model(np.array([np.array([-3,-3])])))

header = ['x', 'y', 'z']
with open('trained.csv', 'w', encoding='UTF8') as f:
  writer = csv.writer(f)

  # write the header
  writer.writerow(header)

  # write the data
  for i in range(1000):
    x = random.uniform(-3, 1.4)
    y = random.uniform(-3, 3)
    angle = math.atan2(1.5 - x, -y)
    #z = norm_kick_chance_model(np.array([np.array([x,y,angle,3,0,0,0,-1,0,1,0,0.1,x,y])]))
    z = norm_kick_chance_model(np.array([np.array([x,y])]))
     
    writer.writerow([x, y, z.numpy()[0][0]])

import plot
