import numpy as np
from utils import wrapToPi

def ctrl_pose(x,y,th,x_g,y_g,th_g):
    # (x,y,th): current state
    # (x_g,y_g,th_g): desired final state

    # Code pose controller
    k = np.array([0.5, 0.5, 1.2]) # (k1,k2,k3) > 0

    # Incremental change in state
    dx = x_g - x
    dy = y_g - y

    # Convert to polar coordinates
    rho = np.sqrt(dx**2 + dy**2)
    alpha = wrapToPi(np.arctan2(dy,dx) - th)
    delta = wrapToPi(alpha + th - th_g)

    # Closed-loop control law
    V = k[0]*rho*np.cos(alpha)
    om = k[1]*alpha + k[0]*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha + k[2]*delta)

    # Apply saturation limits
    V = np.sign(V)*min(0.5, np.abs(V))
    om = np.sign(om)*min(1, np.abs(om))

    return np.array([V, om])
