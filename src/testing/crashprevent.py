import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button, Slider

def calculate_angle(theta0, theta1, theta2) -> float:
    numerator = 10-((57.44 + 65.56)*np.sin(theta0) + 170*np.sin(theta0+theta1) + 80*np.sin(theta0+theta1+theta2))
    denominator = 124.08 + 82.18
    x = numerator / denominator
    t = np.arcsin(x)   
    theta_max = t - theta0 - theta1 -theta2
    return theta_max

def get_z(theta0, theta1, theta2, theta3):
    x = (57.44 + 65.56)*np.sin(theta0) + 170*np.sin(theta0 + theta1)+ 80*np.sin(theta0 + theta1 + theta2)+ (124.08 + 82.18)*np.sin(theta0 + theta1 + theta2 + theta3)
    return x

def get_r(theta0, theta1, theta2, theta3):
    y = (57.44 + 65.56)*np.cos(theta0) + 170*np.cos(theta0 + theta1)+ 80*np.cos(theta0 + theta1 + theta2)+ (124.08 + 82.18)*np.cos(theta0 + theta1 + theta2 + theta3)
    return y

def get_xyz(thetax, theta0, theta1, theta2, theta3):
    z = (57.44 + 65.56)*np.sin(theta0) + 170*np.sin(theta0 + theta1)+ 80*np.sin(theta0 + theta1 + theta2)+ (124.08 + 82.18)*np.sin(theta0 + theta1 + theta2 + theta3)
    r = (57.44 + 65.56)*np.cos(theta0) + 170*np.cos(theta0 + theta1)+ 80*np.cos(theta0 + theta1 + theta2)+ (124.08 + 82.18)*np.cos(theta0 + theta1 + theta2 + theta3)
    x = r * np.cos(thetax)
    y = r * np.sin(thetax)
    return x,y,z




'''
# Inverse Kinematik
def get_thetas(x,y, theta0, theta2):
    x- (57.44 + 65.56)*np.sin(theta0) = 170*np.sin(theta0 + theta1)+ 80*np.sin(theta0 + theta1 + theta2)+ (124.08 + 82.18)*np.sin(theta0 + theta1 + theta2 + theta3)
    y-  (57.44 + 65.56)*np.cos(theta0) =  170*np.cos(theta0 + theta1)+ 80*np.cos(theta0 + theta1 + theta2)+ (124.08 + 82.18)*np.cos(theta0 + theta1 + theta2 + theta3)

    return theta1, theta3
'''

ll0 = 65.56+57.44
ll1 = 170
ll2 = 80
ll3 = 124.08+82.18
def get_thetas(x,y,z, theta0, theta2, ll0, ll1, ll2, ll3):
    l1 = np.sqrt(np.square(ll1) + np.square(ll2))
    l2 = ll3
    angle2 = np.arccos(-(np.square(x)+np.square(y)-np.square(l1)-np.square(l2))/(2*l1*l2))
    phi = np.arccos((np.square(x)+np.square(y)+np.square(l1)-np.square(l2))/(2*l1*np.sqrt(np.square(x)+np.square(y))))
    beta = np.arctan2(y,x)
    angle1 = phi + beta
    theta3 = angle2 + np.pi
    theta1 = np.arctan2(ll2,ll1)+angle1+3/2*np.pi
    return theta1, theta3

                




thetax = 0.7853981633974483#0 # Shoulder joint
theta0 = np.pi/2 # Base fest 90
theta1 = 0.20702981374818408#3/2*np.pi + np.pi/4 
theta2 = 3/2*np.pi # fest 270

# returns theta that sets endpoint on 0
theta3 = -0.7684290901637172#calculate_angle(theta0=theta0, theta1=theta1, theta2=theta2)

anglex = thetax
angle0 = theta0
angle1 = theta0 + theta1
angle2 = theta0 + theta1 + theta2
angle3 = theta0 + theta1 + theta2 + theta3

base = np.array([0*np.cos(angle0)+11.6,65.56+57.44*np.sin(angle0)])
upperarm1 = base + np.array([170*np.cos(angle1), 170*np.sin(angle1)])
upperarm2 = upperarm1 + np.array([80*np.cos(angle2), 80*np.sin(angle2)])
lowerarm = upperarm2 + np.array([(124.08)*np.cos(angle3), (124.08)*np.sin(angle3)])
gripper = lowerarm + np.array([(74.68)*np.cos(angle3), (74.68)*np.sin(angle3)])

z = get_z(theta0=theta0, theta1=theta1, theta2=theta2, theta3=theta3)
r = get_r(theta0=theta0, theta1=theta1, theta2=theta2, theta3=theta3)
print(f"R: {z}, Z: {r}")

# plot setup
'''
y
^
|
|
+------> x
'''
plt.figure()    
plt.axhline(0, color='gray')
plt.axvline(0, color='gray')
plt.grid()

# draw vectors
plt.quiver(11.6, 0, base[0]-11.6, base[1], angles='xy', scale_units='xy', scale=1, color='r', label='base')
plt.quiver(base[0], base[1], upperarm1[0]-base[0], upperarm1[1]-base[1],
           angles='xy', scale_units='xy', scale=1, color='b', label='upperarm1')
plt.quiver(upperarm1[0], upperarm1[1], upperarm2[0]-upperarm1[0], upperarm2[1]-upperarm1[1],
           angles='xy', scale_units='xy', scale=1, color='g', label='upperarm2')
plt.quiver(upperarm2[0], upperarm2[1], lowerarm[0]-upperarm2[0], lowerarm[1]-upperarm2[1],
           angles='xy', scale_units='xy', scale=1, color='r', label='lowerarm')
plt.quiver(lowerarm[0], lowerarm[1], gripper[0]-lowerarm[0], gripper[1]-lowerarm[1],
           angles='xy', scale_units='xy', scale=1, color='y', label='gripper')


plt.xlim(-400, 400)
plt.ylim(-400, 400)
plt.legend()
plt.gca().set_aspect('equal')
plt.show(block=False)

# =============================
# =============================

plt.style.use('_mpl-gallery')
# Make data
thetax =0.7853981633974483#np.pi/4 # Shoulder joint
theta0 = np.pi/2 # Base fest 90
theta1 =  0.18817863736154722#3/2*np.pi + np.pi/4 # Upperarm
theta2 = 3/2*np.pi # fest 270


# returns theta that sets endpoint on x
theta3 = -0.19892412747510874# calculate_angle(theta0=theta0, theta1=theta1, theta2=theta2) # Lowerarm
print(f"Angles: {thetax, theta0, theta1, theta2, theta3}")

anglex = thetax
angle0 = theta0
angle1 = theta0 + theta1 
angle2 = theta0 + theta1 + theta2
angle3 = theta0 + theta1 + theta2 + theta3

base = np.array([0*np.cos(angle0)*np.cos(anglex),0*np.cos(angle0)*np.sin(anglex),65.56+57.44*np.sin(angle0)])
upperarm1 = base + np.array([170*np.cos(angle1)*np.cos(anglex),170*np.cos(angle1)*np.sin(anglex), 170*np.sin(angle1)])
upperarm2 = upperarm1 + np.array([80*np.cos(angle2)*np.cos(anglex),80*np.cos(angle2)*np.sin(anglex), 80*np.sin(angle2)])
lowerarm = upperarm2 + np.array([(124.08)*np.cos(angle3)*np.cos(anglex),(124.08)*np.cos(angle3)*np.sin(anglex), (124.08)*np.sin(angle3)])
gripper = lowerarm + np.array([(82.18)*np.cos(angle3)*np.cos(anglex),(82.18)*np.cos(angle3)*np.sin(anglex), (82.18)*np.sin(angle3)])

x = np.cos(thetax)*(0*np.cos(theta0)+170*np.cos(theta0 + theta1)+80*np.cos(theta0 + theta1 + theta2)+(124.08+82.18)*np.cos(theta0 + theta1 + theta2 + theta3))
y = np.sin(thetax)*(0*np.cos(theta0)+170*np.cos(theta0 + theta1)+80*np.cos(theta0 + theta1 + theta2)+(124.08+82.18)*np.cos(theta0 + theta1 + theta2 + theta3))
z = 65.56+57.44*np.sin(theta0)+ 170*np.sin(theta0 + theta1)+80*np.sin(theta0 + theta1 + theta2)+(124.08+82.18)*np.sin(theta0 + theta1 + theta2 + theta3)

#x = A*(170*np.cos(pi/2 + theta1)+80*np.cos(pi/2 + theta1 + 3/2pi)+(124.08+82.18)*np.cos(pi/2 + theta1 + 3/2pi + theta3))
#y = B*(170*np.cos(pi/2 + theta1)+80*np.cos(pi/2 + theta1 + 3/2pi)+(124.08+82.18)*np.cos(pi/2 + theta1 + 3/2pi + theta3))
#z = 65.56+57.44*np.sin(pi/2)+ 170*np.sin(pi/2 + theta1)+80*np.sin(pi/2 + theta1 + 3/2pi)+(124.08+82.18)*np.sin(pi/2 + theta1 + 3/2pi + theta3)




# Richtungen
p0 = np.array([11.6, 0, 0])
v0 = base - p0
v1 = upperarm1 - base
v2 = upperarm2 - upperarm1
v3 = lowerarm - upperarm2
v4 = gripper - lowerarm

# get xyz of endpoint
x,y,z = get_xyz(thetax=thetax, theta0=theta0, theta1=theta1, theta2=theta2, theta3=theta3)
print(f"X,Y,Z: {x,y,z}")
print(f"Gripper Endpoint: {gripper}")

ll0 = 65.56+57.44
ll1 = 170
ll2 = 80
ll3 = 124.08+82.18
theta1, theta3 = get_thetas(x,y,z, theta0, theta2, ll0, ll1, ll2, ll3)
print(f"Angles 2: {thetax, theta0, theta1, theta2, theta3}")

# ==== Plot ==== 
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

ax.quiver(11.6, 0, 0, v0[0], v0[1], v0[2], color='r', arrow_length_ratio=0.1, linewidth=5)
ax.quiver(base[0], base[1], base[2], v1[0], v1[1], v1[2], color='k', arrow_length_ratio=0.0, linewidth=4)
ax.quiver(upperarm1[0], upperarm1[1], upperarm1[2], v2[0], v2[1], v2[2], color='k', arrow_length_ratio=0.1, linewidth=4)
ax.quiver(upperarm2[0], upperarm2[1], upperarm2[2], v3[0], v3[1], v3[2], color='b', arrow_length_ratio=0.1, linewidth=3)
ax.quiver(lowerarm[0], lowerarm[1], lowerarm[2], v4[0], v4[1], v4[2], color='g', arrow_length_ratio=0.1, linewidth=2)
ax.scatter(11.6, 0, 0, color='red', s=50)

ax.set_xlabel('X == X == X')
ax.set_ylabel('Y == Y == Y')
ax.set_zlabel('Z == Z == Z')

ax.set_xlim(-400, 400)
ax.set_ylim(-400, 400)
ax.set_zlim(0, 400)

plt.show(block=False)

# =======
# Test inverse kinematic

ll0 = 65.56+57.44
ll1 = 170
ll2 = 80
ll3 = 124.08+82.18

def get_thetas2(x,y,z, l1, l2):
    angle2 = np.arccos(-(np.square(x)+np.square(y)-np.square(l1)-np.square(l2))/(2*l1*l2))+3/2*np.pi
    phi = np.arccos((np.square(x)+np.square(y)+np.square(l1)-np.square(l2))/(2*l1*np.sqrt(np.square(x)+np.square(y))))
    beta = np.arctan2(y,x)
    angle1 = phi + beta
    return angle1, angle2

l1 = 100
l2 = 100
x,y,z = 100,50,0

theta11, theta22 = get_thetas2(x,y,z,l1, l2)
print(f"Angles 3: {theta11, theta22}")


# plot setup
plt.figure()    
plt.axhline(0, color='gray')
plt.axvline(0, color='gray')
plt.grid()

# draw vectors
plt.quiver(0, 0, l1*np.cos(theta11),l1*np.sin(theta11), angles='xy', scale_units='xy', scale=1, color='r', label='base')
plt.quiver(l1*np.cos(theta11), l1*np.sin(theta11), l2*np.cos(theta22), l2*np.sin(theta22),
           angles='xy', scale_units='xy', scale=1, color='b', label='upperarm1')


plt.xlim(-400, 400)
plt.ylim(-400, 400)
plt.legend()
plt.gca().set_aspect('equal')
plt.show()
