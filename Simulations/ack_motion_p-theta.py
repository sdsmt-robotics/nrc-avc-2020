import numpy as np
import matplotlib.pyplot as plt

L = 0.36       # m
r = 0.07       # m
robot_d = 1    # m
robot_d_sq = robot_d**2
dt = 0.1       # s
v_max = 2      # m/s
v_min = 1      # m/s
v_ramp = 0.1   # m/s/s
p = 50

def fk_dt(phi, v, x_k=0, y_k=0, t_k=0):
    x_k = x_k + r * dt * v * np.cos(t_k)
    y_k = y_k + r * dt * v * np.sin(t_k)
    t_k = t_k + r * dt * v * np.tan(phi) / L

    return [x_k, y_k, t_k]


# Returns the angle difference between the current trajectory and the goal, measured CCW from the current trajectory
def theta_error(x, y, t, x_d, y_d):
    t_goal = np.arctan2(y_d - y, x_d - x)
    e = t_goal - t
    ## CRITICAL: ENSURE THAT THE ERROR IS BETWEEN -PI, PI OTHERWISE IT BEHAVES WEIRD
    if e > np.pi:
        e = np.pi * 2 - e
    elif e < -np.pi:
        e = np.pi * 2 + e
    return e


def p_ik(e, p, v):
    phi = np.arctan2(r*L*p*e, v)
    return phi


# current_point stores the index of the current point
# loc = [x, y, theta] current position
# while current_point < len(points):
    # if current_loc > len(points) - 1:
        # if v > (v_min + v_ramp):
            # v = v - v_ramp
    # elif v < (v_min - v_ramp):
        # v = v + v_ramp
    # e = theta_error()
    # w1, w2 = p_ik
    # loc = fk_dt
    # d = sqrt(points(current_point) - loc)
    # if d < robot_d:
        # current_point += 1

points = 2 * np.array([(0, 0), (1, 1), (2, 0), (3, -1), (4, 0), (5, 1), (4,2), (3, 3), (2, 3), (1, 2), (0, 1), (0, 0)])
for i in range(points.shape[0]):
    plt.plot(points[i][0], points[i][1], marker='.', color='k')
current_point = 0
loc = [0, 0, 0] # Current position, in [x, y, theta]
phi = 0
x = []
y = []
theta = []
time = [0]
v = v_max
while current_point < points.shape[0] - 1:
    ## Compute current position based on last time step and measurement
    loc = fk_dt(phi, v, loc[0], loc[1], loc[2])

    ## Compute the angle error
    e = theta_error(loc[0], loc[1], loc[2], points[current_point + 1][0], points[current_point + 1][1])
    phi = p_ik(e, p, v)

    ## Determine if we passed the obstacle
    d = (points[current_point + 1][0] - loc[0])**2 + (points[current_point + 1][1] - loc[1])**2
    if d < robot_d_sq:
        current_point += 1

    ## Plotting
    x.append(loc[0])
    y.append(loc[1])
    theta.append(loc[2])
    time.append(time[len(time) - 2] + dt)
    if len(x) % 10 == 0:
        plt.plot(loc[0], loc[1], marker='.', color='b')
        plt.show(block=False)
        plt.pause(0.1)

plt.plot(x, y)
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.title('DD robot driving an obstacle course')
plt.show()