# Import packages
import numpy as np
import scipy
# Define constants
g=9.81 #units: m/s^2 #Acceleration due to gravity


M=.1 #units: kg #Robot body mass (not including wheels)
M_w=.035*2 #units: kg #Wheel mass
L=.15 #units: meters #Robot beam length
r=.033 #units: meters #Wheel radius
I_w=.5*M_w*r**2 #units: kg-m^2 #Moment of inertia for wheel #Idealizes wheel as solid disk
I_b=M*L**2 #units: kg-m^2 #Moment of inertia for robot body around pivot point #Assumes all mass at top which is somewhat realistic because the heaviest component is the battery pack mounted on the top
## Define state space
# Define constant c (inverse of the determinant of mass matrix)
Mass_matrix = np.array([[M*L, M*L**2+I_b],[M+M_w+I_w/r**2, M*L]])
c = 1/np.linalg.det(Mass_matrix)
# State space vector: [x, x_dot, theta, theta_dot]T
A = np.array([[0,1,0,0],[0,0,c*g*(M*L)**2,0],[0,0,0,1],[0,0,-c*M*g*L*(M+M_w+I_w/r**2),0]])
B = np.array([[0],[-c*(M*L+M*L**2/r+I_b/r)], [0], [c*(M+M_w+I_w/r**2+M*L/r)]])
## Assess controllability of state space
control_matrix = np.column_stack((B, A@B,A@A@B,A@A@A@B))
control_rank = np.linalg.matrix_rank(control_matrix)
print(control_rank)
# control_rank shows state space is fully controllable if it equals 4

## LQR
# Choose state-error cost (Q) and control-effort cost (R)
Q = np.array([[.0001, 0,0,0],[0,.0001,0,0],[0,0,1,0],[0,0,0,.1]])
R = np.array([[1]])
# Determine LQR feedback gain matrix (K)
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R)@np.linalg.matrix_transpose(B)@P
print(K)

# Check eigenvalues. If all eigenvalues have negative real parts then closed-loop system is stable
A_cl = A-B@K
eigs = np.linalg.eigvals(A_cl)
print(eigs)

