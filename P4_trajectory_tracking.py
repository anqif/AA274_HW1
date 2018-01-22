import numpy as np
from numpy import linalg
from P3_pose_stabilization import ctrl_pose

def ctrl_traj(x,y,th,ctrl_prev,x_d,y_d,xd_d,yd_d,xdd_d,ydd_d,x_g,y_g,th_g):
    # (x,y,th): current state
    # ctrl_prev: previous control input (V,om)
    # (x_d, y_d): desired position
    # (xd_d, yd_d): desired velocity
    # (xdd_d, ydd_d): desired acceleration
    # (x_g,y_g,th_g): desired final state

    # Timestep
    dt = 0.005

    # Gains
    kpx = 1
    kpy = 1
    kdx = 0.5
    kdy = 0.5

    # Cutoffs
    d_eps = 0.5   # Switch to pose controller
    V_eps = 1e-5  # Reset to nominal velocity

    # Define control inputs (V,om) - without saturation constraints
    # Switch to pose controller once "close" enough, i.e., when
    # the robot is within 0.5m of the goal xy position.
    if np.sqrt((x_g-x)**2 + (y_g-y)**2) <= d_eps:
        return ctrl_pose(x,y,th,x_g,y_g,th_g)

    # Assume current velocity is that commanded in previous timestep
    V_prev = ctrl_prev[0]
    x_dot = V_prev*np.cos(th)
    y_dot = V_prev*np.sin(th)
    
    # Virtual control law
    u1 = xdd_d + kpx*(x_d-x) + kdx*(xd_d-x_dot)
    u2 = ydd_d + kpy*(y_d-y) + kdy*(yd_d-y_dot)

    # Recover actual control inputs
    V_dot = u1*np.cos(th) + u2*np.sin(th)
    V = V_prev + dt*V_dot # Euler step

    # Reset to desired velocity if V = 0
    if np.abs(V) <= V_eps:
        V = np.sqrt(xd_d**2 + yd_d**2)
        print "Resetting to nominal velocity: ", V
    om = (-u1*np.sin(th) + u2*np.cos(th))/V

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
