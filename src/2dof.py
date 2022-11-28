#!/usr/bin/env python3
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from math import cos,sin,radians,sqrt,acos,pi
import matplotlib.animation as animation
import pickle

l1 = 2

# dh matrix in the form theta, alpha, a, and d
dh = [[0, 0,l1,0],
      [0, 0,l1,0],
      [0, 0,l1,0],
      [0, 0, l1, 0],
      [0, 0, l1, 0]]

target = [5,3,0]

def read_data():
    graph_data = open('target.txt','r').read()
    lines = graph_data.split('\n')
    for line in lines:
        if len(line) > 1:
            x, y,z = line.split(',')
            target = [float(x),float(y),float(z)]
            return target

class Robot:
    def __init__(self,dh):
        self.dh = dh
        self.X = [0]
        self.Y = [0]
        self.Z = [0]
    def trf_matrix(self):
        trf = np.eye(4)
        for th,al,a,d in self.dh:
                T = np.array([
                [cos(th), -cos(al)*sin(th), sin(al)*sin(th), a*cos(th)],
                [sin(th), cos(al)*cos(th), -sin(al)*cos(th), a*sin(th)],
                [0,          sin(al),          cos(al),          d    ],
                [0,            0,                 0,             1    ]
                ])
                trf = np.matmul(trf,T)
                yield trf
    def forward(self):
        self.X = [0]
        self.Y = [0]
        self.Z = [0]
        for t in self.trf_matrix():
            X = np.array([[0],[0],[0],[1]])
            X2 = np.matmul(t,X)
            self.X.append(X2[0,0])
            self.Y.append(X2[1,0])
            self.Z.append(X2[2,0])

    def get_angle(self,a,b,c):
        v1 = a-b
        v2 = c-b

        v1norm = v1/sqrt(np.sum(v1**2))
        v2norm = v2/sqrt(np.sum(v2**2))

        dot = np.dot(v1norm,v2norm)
        cross = np.cross(v1norm[:2],v2norm[:2])
        try:
            angle = acos(dot)
        except:
            print("unable to calculate angle")
            return None
        if cross>0:
            angle = -angle
        #print(angle)
        return angle

    def ccd(self,target,i):
        #print(i)
        dist = lambda a,b: sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)
        error = dist([self.X[-1],self.Y[-1],self.Z[-1]],target)
        if error>1e-3:
            print(error)
            a = np.array(target)
            c = np.array([self.X[-1],self.Y[-1],self.Z[-1]])
            b = np.array([self.X[i],self.Y[i],self.Z[i]])
            an = self.get_angle(a,b,c)
            if an!=None:
                self.dh[i][0] = (self.dh[i][0]+an)
                self.forward()
        else:
            print("target reached")

rob = Robot(dh)

fig = plt.figure()
ax1 = plt.subplot(1,1,1)

theta =0
j = len(dh)-1
def animate(i):
    target = read_data()
    rob.forward()
    if i !=0:
        global j
        rob.ccd(target,j)
        j-=1
        if j<0:
            j = len(dh)-1

    ax1.clear()
    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.grid()
    #ax1.set_zlim(0,5)
    ax1.set_xlabel("X axis")
    ax1.set_ylabel("Y axis")
    #ax1.set_zlabel("Z axis")
    ax1.plot(rob.X,rob.Y)
    ax1.scatter(rob.X,rob.Y)
    ax1.scatter(target[0],target[1])
#    plt.ylim(-5,5)
ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
