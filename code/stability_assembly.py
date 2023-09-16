# https://hades.mech.northwestern.edu/index.php/Stability_of_an_Assembly_Project

import numpy as np
import scipy.optimize as opt

## test examples
# unstable constellation
#print("Setting up unstable test constellation")
#bodies =  np.array([
#    [25, 35, 2],
#    [66, 42, 5]])
#contacts = np.array([
#    [1, 2, 60, 60, np.pi   , 0.50],
#    [1, 0, 0 , 0 , np.pi/2., 0.10],
#    [2, 0, 60, 0 , np.pi/2., 0.50],
#    [2, 0, 72, 0 , np.pi/2., 0.50]])

# stable constellation
print("Setting up stable test constellation")
bodies =  np.array([
    [25, 35, 2],
    [66, 42, 10]])
contacts = np.array([
    [1, 2, 60, 60, np.pi, 0.50],
    [1, 0, 0 , 0 , np.pi/2., 0.50],
    [2, 0, 60, 0 , np.pi/2., 0.50],
    [2, 0, 72, 0 , np.pi/2., 0.50]])


## setting up three body example
#mu = 1e-3 # friction coefficient
#print("Setting up three body arch structure example.")
#print("Friction mu=" + str(mu))
#bodies =  np.array([
#    [-30, 15,  4],
#    [ 30, 15,  4],
#    [  0, 30,  4]])
#contacts = np.array([
#    [1, 0, -50,  0, 1./2.*np.pi, mu],
#    [1, 0, -20,  0, 1./2.*np.pi, mu],
#    [2, 0,  50,  0, 1./2.*np.pi, mu],
#    [2, 0,  20,  0, 1./2.*np.pi, mu],
#    [1, 3, -10, 20, 5./4.*np.pi, mu],
#    [1, 3, -30, 40, 5./4.*np.pi, mu],
#    [2, 3,  10, 20, 7./4.*np.pi, mu],
#    [2, 3,  30, 40, 7./4.*np.pi, mu]])

# allocate arrays
Fbody = np.zeros([len(bodies)*3, len(contacts)*2]);
Fbody_ext = np.zeros([len(bodies)*3]);
for body_index, body in enumerate(bodies):
    # calc F_ext and add to matrix
    px = body[0]
    py = body[1]
    fx = 0
    fy = -9.81*body[2]
    m = px*fy - py*fx
    Fbody_ext[body_index*3:body_index*3+3] = np.array([[m,fx,fy]])

    # add joint for every contact that involves body
    for i, joint in enumerate(contacts):
        # get both wrenches of cone from normal angle
        for j, cone_angle in enumerate([-np.arctan(joint[5]), +np.arctan(joint[5])]):
            # calculate Fbody for all values
            if joint[0] == body_index+1 or joint[1] == body_index+1:
                px = joint[2]
                py = joint[3]
                fx = joint[5]*np.cos(joint[4]+cone_angle)
                fy = joint[5]*np.sin(joint[4]+cone_angle)
                m = px*fy-py*fx

                Fi = np.array([m, fx, fy])

                if joint[1] == body_index+1:
                    Fi = -Fi
                Fbody[body_index*3:body_index*3+3,i*2+j] = Fi

# prepare linprog
n_k = np.shape(Fbody)[1]
c = np.ones(n_k)
k_bounds = (0, None)
res = opt.linprog(c.T, A_eq=Fbody, b_eq=-Fbody_ext, bounds=k_bounds)
if res.success:
    print("The assembly is stable. The values for k are")
    print(res.x)
else:
    print("The assembly is unstable.")
    print(res)
