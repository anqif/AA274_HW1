import numpy as np
import math
from numpy import linalg
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt

# Constants
t_f = 15
V_max = 0.5
om_max = 1

# Initial conditions
x_0 = 0
y_0 = 0
V_0 = V_max
th_0 = -np.pi/2

# Final conditions
x_f = 5
y_f = 5
V_f = V_max
th_f = -np.pi/2

# Solve Linear equations:
xd_0 = 0
yd_0 = -1
xd_f = 0
yd_f = -1

A = np.array([[1,   0,      0,         0], 
			  [1, t_f, t_f**2,    t_f**3],
			  [0,   1,      0,         0],
			  [0,   1,  2*t_f,  3*t_f**2]])
b_x = np.array([x_0, x_f, xd_0, xd_f])
b_y = np.array([y_0, y_f, yd_0, yd_f])
x_coef = linalg.solve(A, b_x)
y_coef = linalg.solve(A, b_y)

# Compute traj
dt = 0.005
N = int(t_f/dt)
t = dt*np.array(range(N+1)) # t[0],....,t[N]
t = t.T
data = np.zeros((N+1,9))

# Compute trajectory, store in data, format: [x,y,th,V,om,xd,yd,xdd,ydd]
t_mat = np.stack((np.ones(N+1), t, t**2, t**3), axis=1)
x = np.dot(t_mat, x_coef)
y = np.dot(t_mat, y_coef)

xd = np.dot(t_mat, np.array([x_coef[1], 2*x_coef[2], 3*x_coef[3], 0])) # dx/dt
yd = np.dot(t_mat, np.array([y_coef[1], 2*y_coef[2], 3*y_coef[3], 0])) # dy/dy
xdd = np.dot(t_mat, np.array([2*x_coef[2], 6*x_coef[3], 0, 0])) # d^2x/dt^2
ydd = np.dot(t_mat, np.array([2*y_coef[2], 6*y_coef[3], 0, 0])) # d^2y/dy^2

th = np.arctan2(yd, xd) # tan(theta) = yd/xd
V = np.sqrt(xd**2 + yd**2) # V^2 = xd^2 + yd^2
om = (-xdd*np.sin(th) + ydd*np.cos(th))/V # omega = (-d^2x/dt^2 * sin(theta) + d^2y/dt^2 * cos(theta))/V

data = np.stack((x,y,th,V,om,xd,yd,xdd,ydd), axis=1)

## Re-scaling - Compute scaled trajectory quantities at the N points along the geometric path above
# Compute arc-length s as a function of t (HINT: use the function cumtrapz)
s = cumtrapz(V,t,initial=0)

# Compute V_tilde (HINT: at each timestep V_tilde should be computed as a minimum of
# the original value V, and values required to ensure both constraints are satisfied)
idx = (om != 0)
V_tilde = np.minimum(V,V_max)
V_tilde[idx] = np.minimum(V_tilde[idx], V[idx]/np.abs(om[idx]))

# Compute tau (HINT: use the function cumtrapz)
tau = cumtrapz(1/V_tilde,s,initial=0)

# Compute om_tilde
om_tilde = om*V_tilde/V

# Get new final time
tf_new = tau[-1]

# Generate new uniform time grid
N_new = int(tf_new/dt)
t_new = dt*np.array(range(N_new+1))
t_new = t_new.T

# Interpolate for state trajectory
data_scaled = np.zeros((N_new+1,9))
data_scaled[:,0] = np.interp(t_new,tau,data[:,0]) # x
data_scaled[:,1] = np.interp(t_new,tau,data[:,1]) # y
data_scaled[:,2] = np.interp(t_new,tau,data[:,2]) # th
# Interpolate for scaled velocities
data_scaled[:,3] = np.interp(t_new, tau, V_tilde)   # V
data_scaled[:,4] = np.interp(t_new, tau, om_tilde)  # om
# Compute xy velocities
data_scaled[:,5] = data_scaled[:,3]*np.cos(data_scaled[:,2]) # xd
data_scaled[:,6] = data_scaled[:,3]*np.sin(data_scaled[:,2]) # yd
# Compute xy acclerations
data_scaled[:,7] = np.append(np.diff(data_scaled[:,5])/dt,-V_f*data_scaled[-1,4]*np.sin(th_f)) # xdd
data_scaled[:,8] = np.append(np.diff(data_scaled[:,6])/dt, V_f*data_scaled[-1,4]*np.cos(th_f)) # ydd

# Save trajectory data
np.save('traj_data_differential_flatness',data_scaled)

# Plots
plt.rc('font', weight='bold', size=16)

plt.figure()
plt.plot(data_scaled[:,0], data_scaled[:,1],'k-',linewidth=2)
plt.grid('on')
plt.plot(x_0,y_0,'go',markerfacecolor='green',markersize=15)
plt.plot(x_f,y_f,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')

plt.figure()
plt.subplot(2,1,1)
plt.plot(t, data[:,3:5],linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'])
plt.title('Original')

plt.subplot(2,1,2)
plt.plot(t_new,data_scaled[:,3:5],linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'])
plt.title('Scaled')

plt.figure()
plt.plot(t,s,'b-',linewidth=2)
plt.grid('on')
plt.plot(tau,s,'r-',linewidth=2)
plt.xlabel('Time [s]')
plt.ylabel('Arc-length [m]')
plt.legend(['Original', 'Scaled'])

plt.show()
