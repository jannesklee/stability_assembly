# https://hades.mech.northwestern.edu/index.php/Stability_of_an_Assembly_Project

import numpy as np
import scipy.optimize as opt

bodies =  np.array([[25, 35, 2],[66, 42, 5]])
contacts = np.array([
    [1, 2, 60, 60, np.pi   , 0.50],
    [1, 0, 0 , 0 , np.pi/2., 0.10],
    [2, 0, 60, 0 , np.pi/2., 0.50],
    [2, 0, 72, 0 , np.pi/2., 0.50]])


# set up equation system
for body_index, body in enumerate(bodies):
    Fbody_ext = np.array([[0,0,-9.81*body[2]]])

    # add joint for every contact that involves body
    Fbody = []
    for i, joint in enumerate(contacts):
        # define all wrenches
        if body_index == (joint[0]-1) or body_index == (joint[1]-1):
            px = joint[2]
            py = joint[3]
            fx = joint[5]*np.cos(joint[4])
            fy = joint[5]*np.sin(joint[4])
            m = px*fy-py*fx

            Fi = np.array([m, fx, fy])

            if body_index == (joint[1]-1):
                Fi = -Fi

            Fbody.append(Fi)

    Fbody = np.array(Fbody)
    n_joints = np.shape(Fbody)[0]

    # add ext wrenches
    # constraint
    c = np.ones([1, n_joints])
    b = -np.ones([1, n_joints])
    A = np.identity(n_joints)
    k_bounds = (0, None)
    res = opt.linprog(c.T, A_eq=Fbody.T, b_eq=-Fbody_ext.T, bounds=k_bounds)
    print(res.x)

