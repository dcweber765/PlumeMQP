import time
import csv
import logging
import sys
import serial
import math
import numpy
from numpy import matrix
from numpy import linalg


startTime = time.strftime("%Y-%m-%d_%H:%M:%S")

def ekf(vl, vr, Z, ts, x_i, y_i, theta_i, P): #EKF function
#vl is velocity of left wheel
#vr is velocity of right wheel
#Z is the low pass filter (not used as of 2/1/19)
    width = .35 #meters
    sigmaD = 0
    sigmaTheta = 0
    sigmaX = float(.0001878)
    sigmaY = float(.0002921)
    sigmaOmega = float(.0000036774) #these probs need to change
    R = matrix([[sigmaX,0,0],[0,sigmaY,0],[0,0,sigmaOmega]])

    ts = float(ts)
    xhat = matrix([[x_i, y_i, theta_i]])
    C = matrix([[1/(.001*ts**2),0,0],[0,1/(.001*ts**2),0],[0,0,1/(.001*ts**2)]])

    #linearized change in pose for this discrete time step
    dD = .5*.001*ts*(vl+vr)
    dTh = (.001*ts*(vr-vl))/width

    #state prediction
    xhat_p = matrix([[xhat.item(0)+dD* math.cos(xhat.item(2))], [xhat.item(1)+dD* math.sin(xhat.item(2))], [xhat.item(2)+dTh]])

    #jacobian of f with respect to states
    dFdXhat = matrix([[1,0,-1*dD*math.sin(xhat.item(2)+dTh)],[0,1,dD*math.cos(xhat.item(2)+dTh)],[0,0,1]])

    #jacobian of f with respect to control input
    dFdU = matrix([[-1*dD*math.sin(xhat.item(2)+dTh),math.cos(xhat.item(2)+dTh)],[dD*math.cos(xhat.item(2)+dTh),math.sin(xhat.item(2)+dTh)],[1,0]])
    #Process noise covariance
    Q = dFdU.dot(matrix([[sigmaTheta,0],[0,sigmaD]]).dot(dFdU.transpose()))

    #Process Error Covariance Prediction
    P = dFdXhat.dot(P.dot(dFdXhat.transpose()))+Q

    #Innovation Matrix
    S = C.dot(P.dot(C.transpose()))+R

    #Kalman Gain
    K = P.dot((C.transpose()).dot(linalg.inv(S)))

    #State Update
    xhat = xhat_p+K.dot(Z-(C.dot(xhat_p - xhat)))

    #Process Error Update
    P = (numpy.identity(3)-(K.dot(C)))*P

    return xhat, P


def main():
    print "waiting"
    time.sleep(.25)
    x_i = 0
    y_i = 0
    theta_i = 0
    sigmaTheta = 0
    sigmaX = float(.0001878)
    sigmaY = float(.0002921)
    sigmaOmega = float(.0000036774) #these probs need to change
    P = matrix([[sigmaX,0,0],[0,sigmaY,0],[0,0,sigmaOmega]])# init P matrix as R
    print "GO!"
    #endCon = endCondtion()
    #while not endCon:
    fileName = 'EKF_data_' + startTime + '.csv'
    with open(fileName, mode='w') as csvfile:
        entries = ['X','Y','Theta']
        writer = csv.DictWriter(csvfile, fieldnames=entries)

        writer.writeheader()
        for j in range(5):
            ts = .05
            Z = 
            xhat, P = ekf(vl, vr, ts, x_i, y_i, theta_i, P)
            x_i = xhat.item(0) #position in meters
            y_i = xhat.item(1) #position in meters
            theta_i = xhat.item(2) #angle in rads?
            print xhat
            writer.writerow({'X' : x_i, 'Y' : y_i, 'Theta' : theta_i})
    print "Done!"
    print xhat



if __name__ == "__main__":
    main()
