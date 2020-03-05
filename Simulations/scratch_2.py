import numpy as np
a1 = 7
a2 = 5
for i in range(0, 1, 0.05):
    theta_1 = 3.14159 / 2 * i
    theta_2 = 3.14159 / 2 * i
    x = a2 * np.cos(theta_1 + theta_2) + a1 * np.cos(theta_1)
    y = a2 * np.sin(theta_1 + theta_2) + a1 * np.sin(theta_1)
    print([x, y])