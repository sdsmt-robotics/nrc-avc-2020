import numpy as np
import matplotlib.pyplot as plt
import time

D = 10
L = 16
r = D/2

def fk(w1, w2, t, x_0 = 0, y_0 = 0, theta_0 = 0):
    x = L * (w1 + w2) / (w1 - w2) * (np.sin((r / 2 / L) * (w1 - w2) * t + theta_0) - np.sin(theta_0)) + x_0
    y = L * (w1 + w2) / (w1 - w2) * (np.cos((r / 2 / L) * (w1 - w2) * t + theta_0) - np.cos(theta_0)) + y_0
    theta = (r/2/L)*(w1 - w2)*t

    return [x, y, theta]

def fk_dt(w1, w2, dt):
    
    x_k = 0
    y_k = 0
    t_k = 0
    x_k1 = []
    y_k1 = []
    t_k1 = []
    
    for i in range(np.size(w1)):
        x_k = x_k + r*dt/2*(w1[i] + w2[i])*np.cos(t_k)
        y_k = y_k + r*dt/2*(w1[i] + w2[i])*np.sin(t_k)
        t_k = t_k + r*dt*(w1[i] - w2[i])/(2.0*L)
        x_k1.append(x_k)
        y_k1.append(y_k)
        t_k1.append(t_k)
        #print([x_k, y_k, t_k])

    return [x_k1, y_k1, t_k1]

def draw_plots(x, y):
    # Draws the configuration and workspace plots for a device with:
    # two angles t1 and t2 in radians
    # x-y coordinates in cm
    plt.plot(x, y, lw=2)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Workspace')
    plt.grid(True)

    plt.tight_layout()
    plt.show()
    return

def animate_plots(x, y, total_time=None):
    # Slowly animates the configuration and workspace plots for a device with:
    # two angles t1 and t2 in radians
    # x-y coordinates in cm
    x_permanent = []
    y_permanent = []
    if total_time is not None:
        delay = total_time / np.size(x)
    else:
        delay = 0.1

    for i in range(np.size(x)):
        x_permanent.append(x[i])
        y_permanent.append(y[i])
        plt.plot(x_permanent, y_permanent, 'o', lw=2)
        plt.xlim(0, 100)
        plt.ylim(-15, 0)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title('Workspace')
        plt.grid(True)

        plt.show(block=False)
        plt.pause(delay)
        plt.clf()
    return

def prob_2_29_dt(T=100, plot=1):
    # Roboscience v1.4.5 problem 2.29
    dt = 1 / T

    # Section 1
    t = 5
    w1 = np.full((1, t*T), 2)[0]
    w2 = np.full((1, t*T), 2)[0]
    
    # Section 2
    t = 1
    w1 = np.concatenate([w1, np.full((1, t*T), 3)[0]])
    w2 = np.concatenate([w2, np.full((1, t*T), 4)[0]])
    
    # Section 3
    t = 4
    w1 = np.concatenate([w1, np.full((1, t*T), 1)[0]])
    w2 = np.concatenate([w2, np.full((1, t*T), 2)[0]])
    
    [x, y, t] = fk_dt(w1, w2, dt)
    if plot == 1:
        #animate_plots(x, y, total_time=1)
        draw_plots(x, y)
    n_points = np.size(x) - 1
    print([x[n_points], y[n_points], t[n_points]])
    return


def prob_2_29():
    # Roboscience v1.4.5 problem 2.29

    # Section 1
    t = 5
    w1 = 2
    w2 = 2
    [x, y, theta] = [w1*t*r, 0, 0]

    # Section 2
    t = 1
    w1 = 3
    w2 = 4
    [x1, y1, theta1] = fk(w1, w2, t, theta_0=theta)

    # Section 3
    t = 4
    w1 = 1
    w2 = 2
    [x2, y2, theta2] = fk(w1, w2, t, theta_0=theta1)

    #animate_plots(x, y, total_time=1)
    print([x, y, theta])
    print([x1, y1, theta1])
    print([x2, y2, theta2])
    print([x + x1 + x2, y + y1 + y2, theta + theta1 + theta2])
    #draw_plots(x, y)
    return

prob_2_29()
prob_2_29_dt(T=100, plot=1)
print(0**0)