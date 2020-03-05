import numpy as np

# f(x) = ax^2 + bx + c
a = 5
b = 0
c = 2

# from lb to ub
lb = 0
ub = 6

# number of rectangles
rec = 100

integral = 0
rec_width = (ub - lb) / float(rec)
arr = np.delete(np.linspace(lb, ub, rec + 1), 0)
area_rect = a*(arr**2) + b*arr + c

for i in range(rec):
    integral += area_rect[i]

print(integral * (ub - lb) / rec)

integral = 1/3.0*a*(ub - lb)**3 + 1/2.0*b*(ub - lb)**2 + c*(ub - lb)
print(integral)