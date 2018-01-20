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
    # kpx = #...TODO...#
    # kpy =
    # kdx =
    # kdy =

    # Define control inputs (V,om) - without saturation constraints
    # Switch to pose controller once "close" enough, i.e., when
    # the robot is within 0.5m of the goal xy position.
    if np.sqrt((x_g-x)**2 + (y_g-y)**2) <= 0.5:
        return ctrl_pose(x,y,th,x_g,y_g,th_g)

    # Assume current velocity is that commanded in previous timestep
    V_prev = ctrl_prev[0]
    om_prev = ctrl_prev[1]
    x_dot = V_prev*np.cos(th-om_prev*dt)   # d(th)/dt = (th-th_prev)/dt = om
    y_dot = V_prev*np.sin(th-om_prev*dt)
    
    # Virtual control law
    u1 = xdd_d + kpx*(x_d-x) + kdx*(xd_d-x_dot)
    u2 = ydd_d + kpy*(y_d-y) + kdy*(yd_d-y_dot)

    # Recover actual control inputs
    # TODO: V = Integrate to get back V
    om = (-u1*np.sin(th) + u2*np.cos(th))/V

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
