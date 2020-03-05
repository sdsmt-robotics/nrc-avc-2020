import numpy as np
import matplotlib.pyplot as plt
import time

D = 10
L = 16
r = D/2

def fk_dt(w1, w2, dt, x_k=0, y_k=0, t_k=0):
    x_k = x_k + r * dt / 2 * (w1 + w2) * np.cos(t_k)
    y_k = y_k + r * dt / 2 * (w1 + w2) * np.sin(t_k)
    t_k = t_k + r * dt * (w1 - w2) / (2.0 * L)

    return [x_k, y_k, t_k]


# Returns the angle difference between the current trajectory and the goal, measured CCW from the current trajectory
def theta_error(t, x, y, x_d, y_d):
    t_goal = np.arctan2(y_d - y, x_d - x)
    return t_goal - t


def p_ik(e, p, v):
    w1 = v + e*p
    w2 = v - e*p
    return [w1, w2]


t = []
i_list = []
r_list = []
for i in range(0, 500):
    r = i * np.pi / 180
    e = np.arctan2(np.sin(r), np.cos(r))

    t.append(e)
    i_list.append(i)
    r_list.append(r)

plt.plot(i_list, t)
plt.plot(i_list, r_list)
plt.show()