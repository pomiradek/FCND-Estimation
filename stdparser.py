import numpy as np

gps = np.loadtxt('log/Graph1.txt', delimiter=',', skiprows=1)[:, 1]
acc = np.loadtxt('log/Graph2.txt', delimiter=',', skiprows=1)[:, 1]
print(gps)

gps_std = np.std(gps)
acc_std = np.std(acc)

print('GPS std: ', gps_std)
print('Accelerometer std: ', acc_std)
