import numpy as np
import matplotlib.pyplot as plt
import time

a1 = 15
a2 = 15

def fk(theta_1, theta_2):
    # Computes the X and Y positions of both arms of a two link manipulator centered on the origin
    # from the two angles, theta_1 and theta_2
    # The equations are from Dr. McGough's textbook roboscience v1.4.5
    x1 = np.round(a1 * np.cos(theta_1), 4)
    y1 = np.round(a1 * np.sin(theta_1), 4)
    x2 = np.round(a2 * np.cos(theta_1 + theta_2) + a1 * np.cos(theta_1), 4)
    y2 = np.round(a2 * np.sin(theta_1 + theta_2) + a1 * np.sin(theta_1), 4)

    #print([x2, y2], [theta_1, theta_2])
    return [x1, x2], [y1, y2]

def ik(x, y):
    # Computes one possible set of the angles theta_1 and theta_2 of a two link manipulator centered on the origin
    # from the x and y positions of the end effector
    # The equations are from Dr. McGough's textbook roboscience v1.4.5
    d = (x**2 + y**2 - a1**2 - a2**2)/(2*a1*a2)
    theta_2 = np.round(np.arctan2(np.sqrt(1 - d**2), d), 4)
    theta_1 = np.round(np.arctan2(y, x) - np.arctan2(a2*np.sin(theta_2), a1 + a2*np.cos(theta_2)), 4)

    #print([x, y], [theta_1, theta_2])
    return theta_1, theta_2

def draw_plots(t1, t2, x, y):
    # Draws the configuration and workspace plots for a device with:
    # two angles t1 and t2 in radians
    # x-y coordinates in cm
    plt.subplot(2, 1, 1)
    plt.plot(t1, t2, '-', lw=2)
    plt.xlim(-2 * np.pi, 2 * np.pi)
    plt.ylim(-2 * np.pi, 2 * np.pi)
    plt.xlabel('theta 1 (rad)')
    plt.ylabel('theta 2 (rad)')
    plt.title('Configuration space')
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(x, y, '-', lw=2)
    plt.xlim(-(a1 + a2), (a1 + a2))
    plt.ylim(-(a1 + a2), (a1 + a2))
    plt.xlabel('x (cm)')
    plt.ylabel('y (cm)')
    plt.title('Workspace')
    plt.grid(True)

    plt.tight_layout()
    plt.show()
    return

def animate_plots(t1, t2, x0, x1, x2, y0, y1, y2, total_time=None):
    # Slowly animates the configuration and workspace plots for a device with:
    # two angles t1 and t2 in radians
    # x-y coordinates in cm
    x_permanent = []
    y_permanent = []
    t1_permanent = []
    t2_permanent = []
    if total_time is not None:
        delay = total_time / np.size(t1)
    else:
        delay = 0.1

    for i in range(np.size(t1)):
        x_permanent.append(x2[i])
        y_permanent.append(y2[i])
        t1_permanent.append(t1[i])
        t2_permanent.append(t2[i])

        x_temp = [x0[i], x1[i], x2[i]]
        y_temp = [y0[i], y1[i], y2[i]]

        plt.subplot(211)
        plt.plot(t1_permanent, t2_permanent, '-', lw=2)
        plt.xlim(-2 * np.pi, 2 * np.pi)
        plt.ylim(-2 * np.pi, 2 * np.pi)
        plt.xlabel('theta 1 (rad)')
        plt.ylabel('theta 2 (rad)')
        plt.title('Configuration space')
        plt.grid(True)

        plt.subplot(212)
        plt.plot(x_permanent, y_permanent, '-', lw=2)
        plt.plot(x_temp, y_temp, 'r-', lw=2)
        plt.xlim(-(a1 + a2), (a1 + a2))
        plt.ylim(-(a1 + a2), (a1 + a2))
        plt.xlabel('x (cm)')
        plt.ylabel('y (cm)')
        plt.title('Workspace')
        plt.grid(True)

        plt.show(block=False)
        plt.pause(delay)
        plt.clf()
    return

def prob_2_10_a():
    # Part a of Roboscience v1.4.5 problem 2.10
    x = np.linspace(0, 25, 50)
    y = 25 - x
    t1, t2 = ik(x, y)

    draw_plots(t1, t2, x, y)
    return

def prob_2_10_b():
    # Part b of Roboscience v1.4.5 problem 2.10
    t = np.linspace(0, np.pi, 50)
    x = 10*np.cos(t) + 15
    y = 10*np.sin(t)
    t1, t2 = ik(x, y)

    draw_plots(t1, t2, x, y)
    return

def prob_2_11():
    # Roboscience v1.4.5 problem 2.11
    # Define a square
    x = np.full((1, 10), 5)[0]
    x = np.concatenate([x, np.linspace(5, 20, 10)])
    x = np.concatenate([x, np.full((1, 10), 20)[0]])
    x = np.concatenate([x, np.linspace(20, 5, 10)])

    y = np.linspace(0, 15, 10)
    y = np.concatenate([y, np.full((1, 10), 15)[0]])
    y = np.concatenate([y, np.linspace(15, 0, 10)])
    y = np.concatenate([y, np.full((1, 10), 0)[0]])

    # Calculate the angles and the re-calculate the arm position from those angles
    [t1, t2] = ik(x, y)
    [x1, x2], [y1, y2] = fk(t1, t2)
    zero = np.full((1, np.size(x)), 0)[0]

    animate_plots(t1, t2, zero, x1, x2, zero, y1, y2)
    draw_plots(t1, t2, x, y)
    return

def prob_2_18():
    # Roboscience v1.4.5 problem 2.18

    # Define the semicircle
    x = np.linspace(15, 5, 50)
    y = 8 + np.sqrt(25 - (x - 10)**2)

    #Define the line at y=8
    x = np.concatenate([x, np.linspace(5, 15, 50)])
    y = np.concatenate([y, np.full((1, 50), 8)[0]])

    # Calculate the angles and the re-calculate the arm position from those angles
    [t1, t2] = ik(x, y)
    [x1, x2], [y1, y2] = fk(t1, t2)
    zero = np.full((1, np.size(x)), 0)[0]

    animate_plots(t1, t2, zero, x1, x2, zero, y1, y2, total_time=0.1)
    draw_plots(t1, t2, x, y)
    return

prob_2_18()