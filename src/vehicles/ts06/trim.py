import os, sys
import numpy as np
import matplotlib.pyplot 
from scipy import optimize
from scipy.optimize import minimize
from scipy.optimize import Bounds

# Ensure 'src' is in sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.abspath(os.path.join(current_dir, "..", ".."))
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)



from vehicles.ts06.parameters import Ts06Parameters
from vehicles.ts06.forces import forces
from dynamics.quaternion import Quaternion




def eval(x):

    [theta0,u13,u24] = x

    vCruise = 42.0 # NED

    w = Quaternion.euler2quat(np.asarray([0.0,np.deg2rad(theta0),0.0]))
    v = np.asarray([vCruise,0,0]) @ Quaternion.Mfg(w).T 

    y0 = np.hstack([np.asarray([0.0,0.0,0.0]),w,v,np.asarray([0.0,0.0,0.0])])



    u = np.asarray([u13,u24,u13,u24])

    tau = forces(0,y0,u,np.zeros(6),P)
    f_total, m_total = tau[:3], tau[3:]

    print(np.around(f_total,2),np.around(m_total))

    return np.sum(np.square(f_total)+np.square(m_total))




if __name__ == "__main__":

    P = Ts06Parameters()


    # Initial flight state before trimming

    theta0 = 0.0
    u13 = 0.4  # Upper two props
    u24 = 0.3  # Lower two props

    x0 = np.asarray([theta0,u13,u24]) 


    bounds = np.asarray([[-4, 4], [0.0,1.0], [0.0,1.0]])


    res = minimize(eval, x0, method='nelder-mead', bounds=bounds,options={'xatol': 1e-8, 'disp': True})
    
    print(res.x)