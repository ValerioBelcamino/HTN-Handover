import numpy as np
import matplotlib.pyplot as plt

n_samples = 100
total_time = 5
x = np.arange(0, n_samples, 1)

avg_vel = total_time / n_samples

y = np.ones((n_samples, 1)) * avg_vel

list = []
third = int(n_samples/5)
for i in range(third):
    list.append((1.5*avg_vel)-(i * avg_vel / (2*third)))
for i in range(3*third):
    list.append(avg_vel)
for i in range(third):
    list.append(avg_vel + (i * avg_vel / (2*third)))


fig = plt.figure()
plt.plot(x, y)
plt.plot(x, list)
plt.show()