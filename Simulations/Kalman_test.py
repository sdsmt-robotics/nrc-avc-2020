import numpy as np
import matplotlib.pyplot as plt

N = 100
t = np.linspace(0, 99, 100)
u1 = np.zeros(N)# * np.sin(t) # np.ones((N, 1)) *
u2 = 2 * np.sin(t/25)
mu1, sigma1 = 0.0, 0.05
mu2, sigma2 = 0.0, 0.25
var1 = sigma1 * sigma1
x = np.zeros((N, 2))
x1 = np.zeros((N, 1))
x2 = np.zeros((N, 1))
xf1 = np.zeros((N, 1))
xf2 = np.zeros((N, 1))
F = np.array([[0, 0.1], [-0.02, 0.2]])
FT = F.T
G = np.array([u1, u2]).T

H = np.array([1, 0])
HT = H.T
V = np.array([[var1, 0], [0, var1]])
W = sigma2 * sigma2
P = np.zeros((N, 2, 2))
z = np.zeros(N)
xf = np.zeros((N, 2))

k = 1
while (k<N):
    q = 0#np.random.normal(mu1, sigma1, 2)
    r = 0#np.random.normal(mu2, sigma2, 1)
    x[k] = np.dot(F, x[k-1]) + G[k-1] + q
    z[k] = np.dot(H, x[k]) + r
    x1[k], x2[k] = x[k]
    k = k+1

k = 1
while (k<N):
    xp = np.dot(F, xf[k-1]) + G[k-1]
    pp = np.dot(F, np.dot(P[k-1], FT)) + V
    y = z[k] - np.dot(H ,xp)
    S = np.dot(H, np.dot(pp, HT)) + W
    kal = np.dot(pp, HT) / S
    xf[k] = xp + y*kal
    P[k] = pp - np.outer(kal, np.dot(H, pp))
    xf1[k], xf2[k] = xf[k]
    k = k+1

plt.subplot(211)
plt.plot(t, x1)
plt.plot(t, xf1)
plt.xlabel('time')
plt.ylabel('x[1]')

plt.subplot(212)
plt.plot(t, x2)
plt.plot(t, xf2)
plt.xlabel('time')
plt.ylabel('x[2]')
plt.show()