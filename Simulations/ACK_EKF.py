import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg

path = [[0, 0], [0, 2], [2, 4], [4, 6], [6, 6], [8, 6]]

N = 100
mu1, sigma1 = 0.0, 0.02#0.025
mu2, sigma2 = 0.0, 0.1#0.85
var1 = sigma1*sigma1
var2 = sigma2*sigma2
dt = 1
r = 0.14/2
dd = r*dt
L = 0.36
x = np.zeros((N,3))
z = np.zeros((N,3))
t = np.linspace(0, 10, N)
w = np.full(t.shape, 1)
p = 60*np.cos(t/2.0) * np.pi / 180


k = 1
while (k<N):
  q = np.random.normal(mu1, sigma1, 3)
  r = np.random.normal(mu2, sigma2, 3)
  x[k,0] = x[k-1,0] + dd*w[k]*np.cos(x[k-1,2]) + q[0]
  x[k,1] = x[k-1,1] + dd*w[k]*np.sin(x[k-1,2]) + q[1]
  x[k,2] = x[k-1,2] + dd*np.tan(p[k])/L + q[2]
  z[k,0] = x[k,0] + r[0]
  z[k,1] = x[k,1] + r[1]
  z[k,2] = x[k,2] + r[2]
  k = k+1

H = np.array([[1,0,0],[0,1,0],[0,0,1]])
HT = H.T
V = np.array([[var1,0,0],[0,var1,0],[0,0,var1]])
W = np.array([[var2,0,0],[0,var2,0],[0,0,var2]])
P = np.zeros((N,3,3))
xf = np.zeros((N,3))
xp = np.zeros(3)
sp = np.zeros(3)

k = 1
while (k<N):
  xp[0] = xf[k-1,0] + dd*w[k]*np.cos(xf[k-1,2])
  xp[1] = xf[k-1,1] + dd*w[k]*np.sin(xf[k-1,2])
  xp[2] = xf[k-1,2] + dd*np.tan(p[k])/L
  F1 = [1.0,0.0, -dd*w[k]*np.sin(xf[k-1,2])]
  F2 = [0,1,dd*w[k]*np.cos(xf[k-1,2])]
  F = np.array([F1,F2,[0,0,1]])
  FT = F.T
  pp = np.dot(F,np.dot(P[k-1],FT)) + V
  y = np.dot(H,z[k]) - np.dot(H,xp)
  S = np.dot(H,np.dot(pp,HT)) + W
  SI = linalg.inv(S)
  kal = np.dot(pp,np.dot(HT,SI))
  xf[k] = xp + np.dot(kal,y)
  P[k] = pp - np.dot(kal,np.dot(H,pp))
  k = k+1

t = np.arange(0,N,1)
plt.plot(t, x, 'b-', t,z,'r.', t, xf,'go')
plt.show()

plt.plot(x[:,0], x[:,1], 'b-', label='True')
plt.plot(z[:,0], z[:,1],'r.', label='Measurement')
plt.plot(xf[:,0], xf[:,1], 'go', label='Estimate')
plt.legend()
plt.show()