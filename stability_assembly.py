# https://hades.mech.northwestern.edu/index.php/Stability_of_an_Assembly_Project

import numpy as np
import scipy.optimize as opt

bodies =  np.array([[25, 35, 2],[66, 42, 5]])
contacts = np.array([
    [1, 2, 60, 60, np.pi   , 0.50],
    [1, 0, 0 , 0 , np.pi/2., 0.10],
    [2, 0, 60, 0 , np.pi/2., 0.50],
    [2, 0, 72, 0 , np.pi/2., 0.50]])


for body in bodies:
    # each 
    for joint in joints(body):
        # define all wrenches
        F = np.array([0, 0, -9.81*body[2]])

# add ext wrenches
opt.linprog(c, A_ub, b_ub, A_eq, b_eq, bounds, method, callback, options, x0)

