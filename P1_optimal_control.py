import numpy as np
import math
import scikits.bvp_solver
import matplotlib.pyplot as plt

def z_to_ctrl(z):
    V = -0.5*(z[3]*np.cos(z[2]) + z[4]*np.sin(z[2]))
    om = -0.5*z[5]
    return (V, om)

def q1_ode_fun(tau, z):
    # z = [x, y, th, p1, p2, p3, r]

    # Code in the BVP ODEs
    Vom = z_to_ctrl(z)
    x_d = Vom[0]*np.cos(z[2])
    y_d = Vom[0]*np.sin(z[2])
    th_d = Vom[1]
    p1_d = 0
    p2_d = 0
    p3_d = Vom[0]*(z[3]*np.sin(z[2]) - z[4]*np.cos(z[2]))
    r_d = 0

    return z[6]*np.array([x_d, y_d, th_d, p1_d, p2_d, p3_d, r_d])

def q1_bc_fun(za, zb):

    # lambda
    lambda_test = 0.238

    # goal pose
    x_g = 5
    y_g = 5
    th_g = -np.pi/2.0
    xf = [x_g, y_g, th_g]

    # initial pose
    x0 = [0, 0, -np.pi/2.0]

    # Code boundary condition residuals
    Vom_b = z_to_ctrl(zb)
    BC_tf = lambda_test + Vom_b[0]**2 - zb[4]*Vom_b[0] + Vom_b[1]**2 + zb[5]*Vom_b[1]
    BC_left = za[0:3] - x0
    BC_right = np.append(zb[0:3] - xf, BC_tf)

    return (BC_left, BC_right)

# Define solver state: z = [x, y, th, p1, p2, p3, r]
problem = scikits.bvp_solver.ProblemDefinition(num_ODE = 7, # Number of ODEs
                                               num_parameters = 0, # Number of parameters
                                               num_left_boundary_conditions = 3, # Number of left BCs
                                               boundary_points = (0,1), # Boundary points of rescaled time
                                               function = q1_ode_fun, # ODE function
                                               boundary_conditions = q1_bc_fun) # BC function

soln = scikits.bvp_solver.solve(problem, solution_guess = (0, 0, -np.pi/2.0, 1, 1, 0, 10
                                ))

dt = 0.005

# Test if time is reversed in bvp_solver solution
z_0 = soln(0)
flip = 0
if z_0[-1] < 0:
    t_f = -z_0[-1]
    flip = 1
else:
    t_f = z_0[-1]

t = np.arange(0,t_f,dt)
z = soln(t/t_f)
if flip:
    z[3:7,:] = -z[3:7,:]
z = z.T # solution arranged column-wise

# Recover optimal control histories
Vom = np.apply_along_axis(z_to_ctrl, -1, z)
V = Vom[:,0]
om = Vom[:,1]

V = np.array([V]).T # Convert to 1D column matrices
om = np.array([om]).T

if any(np.abs(V) > 0.5):
    raise ValueError("Control constraint |V(t)| <= 0.5 violated")

if any(np.abs(om) > 1.0):
    raise ValueError("Control constraint |w(t)| <= 1.0 violated")

# Save trajectory data (state and controls)
data = np.hstack((z[:,:3],V,om))
np.save('traj_data_optimal_control',data)

# Plots
plt.rc('font', weight='bold', size=16)

plt.figure()
plt.plot(z[:,0], z[:,1],'k-',linewidth=2)
plt.quiver(z[1:-1:200,0],z[1:-1:200,1],np.cos(z[1:-1:200,2]),np.sin(z[1:-1:200,2]))
plt.grid('on')
plt.plot(0,0,'go',markerfacecolor='green',markersize=15)
plt.plot(5,5,'ro',markerfacecolor='red', markersize=15)
plt.xlabel('X'); plt.ylabel('Y')

plt.figure()
plt.plot(t, V,linewidth=2)
plt.plot(t, om,linewidth=2)
plt.grid('on')
plt.xlabel('Time [s]')
plt.legend(['V [m/s]', '$\omega$ [rad/s]'])

plt.show()
