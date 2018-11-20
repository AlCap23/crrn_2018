# package for many numerical functions
import numpy as np 

# package for plotting data
import matplotlib.pyplot as plt  
from mpl_toolkits.mplot3d import Axes3D

# functions from Modern robotics
import modern_robotics_code as mrc

#####################################################################################

L1 = 0.1; L2 = 0.5; L3 = 0.5

#####################################################################################

q_list_s = [np.array([0,0,0]),np.array([0,L1,0]),np.array([0,L1+L2,0])]
q_list_b = [np.array([0,-(L1+L2),L3]),np.array([0,-L2,L3]),np.array([0,0,L3])]

# Mind that by defining omega, you also define in which direction you have positive rotation!      
# In this case, both omegas are the same
w_list = [np.array([0,0,1]),np.array([1,0,0]),np.array([1,0,0])]

S_list = [] 
for q,w in zip(q_list_s,w_list):
    S_list.append(np.append(w,np.cross(-w,q)))
S_list = np.array(S_list).T

B_list = []
for q,w in zip(q_list_b,w_list):
    B_list.append(np.append(w,np.cross(-w,q)))                                            
B_list = np.array(B_list).T

#####################################################################################

# Given in space frame
M = np.array([[1,0,0,0],
              [0,1,0,L1+L2],
              [0,0,1,-L3],
              [0,0,0,1]])

#####################################################################################

dt = 0.1
t = 0
te = 2

rot = 3.5*np.pi/2/te

location = []
while t < te:
    
    theta1 = -0.6*t/te; theta2 = -0.1*np.cos(rot*t)
    
    T = mrc.FKinSpace(M,S_list,[theta1,theta2,0])
    location.append(T[0:3,3])
    
    t += dt

location = np.array(location)

#####################################################################################

fig1 = plt.figure()
ax = Axes3D(fig1)
ax = fig1.gca(projection='3d')
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_aspect('equal')

# plotting trajectory points
ax.scatter(location[:,0],location[:,1],location[:,2])

# plotting start and end position
ax.scatter(location[0,0],location[0,1],location[0,2],color='r')
ax.scatter(location[-1,0],location[-1,1],location[-1,2],color='g')

# plotting initial frame
ax.scatter(0,0,0,marker="+",linewidth=10)

# this part is used to have a defined plot around our trajectory
X = np.array((-0.6,0.2))
Y = np.array((-0.5,0.8))
Z = np.array((-0.7,0))

max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0

mid_x = (X.max()+X.min()) * 0.5
mid_y = (Y.max()+Y.min()) * 0.5
mid_z = (Z.max()+Z.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()

#####################################################################################


#####################################################################################
input("Press enter to exit ;)")
