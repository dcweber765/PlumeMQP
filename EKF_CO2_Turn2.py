import time
from roboclaw import Roboclaw
import csv
import logging
import sys
import time
import serial
import math
import numpy
from numpy import matrix
from numpy import linalg

from Adafruit_BNO055 import BNO055

bno = BNO055.BNO055()

ser0 = serial.Serial("/dev/ttyUSB0", timeout=1)
ser1 = serial.Serial("/dev/ttyUSB1", timeout=1)
ser2 = serial.Serial("/dev/ttyUSB2", timeout=1)
ser3 = serial.Serial("/dev/ttyUSB3", timeout=1)

#CO2 startup
ser0.flushInput()
ser1.flushInput()
ser2.flushInput()
ser3.flushInput()
time.sleep(1)
ser0.write(b'G\r\n')
ser1.write(b'G\r\n')
ser2.write(b'G\r\n')
ser3.write(b'G\r\n')
print('Zero Point Cal Air')
time.sleep(1)

# RoboClaw Start up
rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80


rc.ResetEncoders(address)
rc.ForwardMixed(address, 0)
rc.TurnRightMixed(address, 0)

# IMU startup
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')



# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

# ROBOT INITIAL CONDITIONS
vl_i = 0
vr_i = 0
x_i = 0
y_i = 0
theta_i = 0
width = .25 #robot width in meters

#STATES + VELOCITIES-
vl = vl_i
vr = vr_i
x = x_i
y = y_i
theta = theta_i

#EKF INITIAL CONDITIONS

#these are R**2 values
sigmaD = 0
sigmaTheta = 0
sigmaX = float(.0001878)
sigmaY = float(.0002921)
sigmaOmega = float(.0000036774)

R = matrix([[sigmaX,0,0],[0,sigmaY,0],[0,0,sigmaOmega]])
P = R;

firstMeas = True

print "Initial Conditions are VL = ", vl_i, ", VR = ", vr_i, ", x = ", x_i, ", y = ,", y_i, ", theta = ", theta_i,


def ekf(vl, vr, Z, ts):
    global xhat
    global P

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

    return xhat


def lowpassfilter(Z, Z_filt):
    Z_filt = (.3758*Z)+(.6242*Z_filt)
    return Z_filt


def turnLeft(speed, goal):
    goal = goal
    heading, roll, pitch = bno.read_euler()
    CurrentHeading = heading
    print "left", goal+CurrentHeading
    while not(CurrentHeading+goal-5 <= heading <= CurrentHeading+goal+5): #+
        heading, roll, pitch = bno.read_euler()
        #print goal
        #print heading
        rc.TurnLeftMixed(address, 10)
    rc.TurnLeftMixed(address,0)
    #time.sleep(.25)

def turnRight(speed, goal):
    goal = goal
    heading, roll, pitch = bno.read_euler()
    CurrentHeading = heading
    print "Rigth", goal-CurrentHeading
    while not(CurrentHeading-goal-5 <= heading <= CurrentHeading-goal+5): #-
        heading, roll, pitch = bno.read_euler()
        #print goal
        #print heading
        rc.TurnRightMixed(address, 10)
    rc.TurnRightMixed(address, 0)
    #time.sleep(.25)

def C02Angle(fltCo20,fltCo21,fltCo22,fltCo23):

    D = 15.5
    c = 11.25
    B = math.pi/4


    inital = [fltCo20,fltCo21,fltCo22,fltCo23]


    sortedvalues2 = sorted(inital)
    i = 0
    sortedvalues = [sortedvalues2[i+3],sortedvalues2[i+2],sortedvalues2[i+1],sortedvalues2[i]]
    position = [1,2,3,4]
    i = 0;
    while i < 4:
        if sortedvalues[i] == fltCo20:
            position[i] = 1

        elif sortedvalues[i] == fltCo21:
            position[i] = 2

        elif sortedvalues[i] == fltCo22:
            position[i] = 3

        elif sortedvalues[i] == fltCo23:
            position[i] = 4
            i = i + 1


	i = 1;

	if position[i] == 1 & position[i+1] == 2:
		goalie = angle(fltCo20,fltCo21)
        turnLeft(32,angle(sortedvalues[0],sortedvalues[1]))
        break

	if position[i] == 1 & position[i+1] == 4:
		goal2 = angle(fltCo20,fltCo23)
        turnRight(32,angle(fltCo20,fltCo23))
        break

	if position[i] == 2 & position[i+1] == 1:
		goal2 = 90-angle(fltCo21,fltCo20)
        turnLeft(32,90-angle(fltCo21,fltCo20))
        break

	if position[i] == 2 & position[i+1] == 3:
		goal2 = 90+angle(fltCo21,fltCo22)
        turnLeft(32,90+angle(fltCo21,fltCo22))
        break

	if position[i] == 3 & position[i+1] == 2:
		goal2 = 180-angle(fltCo22,fltCo21)
        turnLeft(32,180-angle(fltCo22,fltCo21))
        break

	if position[i] == 3 & position[i+1] == 4:
		goal2 = 180-angle(fltCo22,fltCo23)
        turnRight(32,180-angle(fltCo22,fltCo23))
        break

	if position[i] == 4 & position[i+1] == 1:
		goal2 = 90-angle(fltCo23,fltCo20)
        turnRight(32,90-angle(fltCo23,fltCo20))
        break

	if (position[i] == 4 & position[i+1] == 3):
		goal2 = 90+angle(fltCo23,fltCo22)
        turnRight(32,90+angle(fltCo23,fltCo22))
        break
#        goal = heading

def angle(sensor1,sensor2):
    inital = [fltCo20,fltCo21,fltCo22,fltCo23]
    sortedvalues2 = sorted(inital)
    i = 0
    sortedvalues = [sortedvalues2[i+3],sortedvalues2[i+2],sortedvalues2[i+1],sortedvalues2[i]]

    print(sortedvalues)
    i = 0
    D = 15.5
    c = 11.25
    sensor2 = sortedvalues[i+1]
    sensor1 = sortedvalues[i]
    print(sensor1)
    print(sensor2)

    B = (math.pi)/4
    a = sensor2*D / (sensor2 + sensor1)

    b =math.sqrt(a**2+c**2-(2*a*c*math.cos(B)))
    A = math.acos((-a**2+b**2+c**2)/(2*b*c))
    print(A*(180/math.pi))
    return A*(180/math.pi)


def displayspeed():
	enc1 = rc.ReadEncM1(address)
	enc2 = rc.ReadEncM2(address)
	speed1 = rc.ReadSpeedM1(address)
	speed2 = rc.ReadSpeedM2(address)
    #one rotation is 1253
	print("Encoder1:"),
	if(enc1[0]==1):
		print enc1[1],
		print format(enc1[2],'02x'),
	else:
		print "failed",
	print "Encoder2:",
	if(enc2[0]==1):
		print enc2[1],
		print format(enc2[2],'02x'),
	else:
		print "failed " ,
	print "Speed1:",
	if(speed1[0]):
		print speed1[1],
	else:
		print "failed",
	print("Speed2:"),
	if(speed2[0]):
		print speed2[1]
	else:
		print "failed "

def writeSensorData():
    heading, roll, pitch = bno.read_euler()
    accelX, accelY, accelZ  = bno.read_accelerometer()
    gyroX, gyroY, gyroZ = bno.read_gyroscope()
    LinAccelX, LinAccelY, LinAccelZ = bno.read_linear_acceleration()
    ser0.write(b'Z\r\n')
    resp0 = (ser0.read(10))
    resp0 = resp0[:8]
    fltCo20 = float(resp0[2:])
    ser1.write(b'Z\r\n')
    resp1 = ser1.read(10)
    resp1 = resp1[:8]
    fltCo21 = float(resp1[2:])
    ser2.write(b'Z\r\n')
    resp2 = ser2.read(10)
    resp2 = resp2[:8]
    fltCo22 = float(resp2[2:])
    ser3.write(b'Z\r\n')
    resp3 = ser3.read(10)
    resp3 = resp3[:8]
    fltCo23 = float(resp3[2:])
    displayspeed()
    # enc1 = rc.ReadEncM1(address) #Encoder Lines
    # enc2 = rc.ReadEncM2(address)
    # speed1 = rc.ReadSpeedM1(address)
    # speed2 = rc.ReadSpeedM2(address)
    # print 'Encoder1 = ',enc2[2]
    # print 'Encoder2 = ',enc1[2]
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
    print('accelX={0:0.2F} accelY={1:0.2F} accelZ={2:0.2F}'.format(accelX, accelY, accelZ))
    print 'CO2 PPM   =', fltCo20 *10
    print 'CO2 PPM 2 =', fltCo21 *10 # multiplierZ
    print 'CO2 PPM 3 =', fltCo22 *10 # multiplierZ
    print 'CO2 PPM 4 =', fltCo23 *10 # multiplierZ

def goForwardDistance(inch):

    rc.ResetEncoders(address)
    rc.ForwardMixed(address, 0)
    rc.TurnRightMixed(address, 0)
    diameter = 2.975
    circ = diameter*math.pi
    #countPerInch = 1253/(2.975*math.pi)

    #INCHdist = feet/12

#    while distanceAVG < inch:
#        #writeSensorData()
#        rc.ForwardMixed(address, 45)
#        enc1 = rc.ReadEncM1(address)
#    	enc2 = rc.ReadEncM2(address)
#        distanceL = circ*enc1[1]/1253
#        distanceR = circ*enc2[1]/1253

#        distanceAVG = (distanceL+distanceR)/2
#        print distanceAVG

    rc.ForwardMixed(address, 0)
    rc.TurnRightMixed(address, 0)

print('Recording BNO055 data')
print('Recoding CO2 data')


i = 0

while(i < 2):
    #rc.ForwardMixed(address, 45)
    #goForwardDistance(18)
    for j in range(10):
        speed1 = rc.ReadSpeedM1(address)[1]
        speed2 = rc.ReadSpeedM2(address)[1]
        accelX, accelY, accelZ  = bno.read_accelerometer()
        gyroX, gyroY, gyroZ = bno.read_gyroscope()
        Z = matrix([[float(accelX)], [float(accelY)], [float(gyroZ)]])
        ts = .05
        ekf(speed1, speed2, Z, ts)
        #rc.ForwardMixed(address, 45)
        writeSensorData()
        ser0.write(b'Z\r\n')
        resp0 = (ser0.read(10))
        resp0 = resp0[:8]
        fltCo20 = float(resp0[2:])
        ser1.write(b'Z\r\n')
        resp1 = ser1.read(10)
        resp1 = resp1[:8]
        fltCo21 = float(resp1[2:])
        ser2.write(b'Z\r\n')
        resp2 = ser2.read(10)
        resp2 = resp2[:8]
        fltCo22 = float(resp2[2:])
        ser3.write(b'Z\r\n')
        resp3 = ser3.read(10)
        resp3 = resp3[:8]
        fltCo23 = float(resp3[2:])
        C02Angle(fltCo20,fltCo21,fltCo22,fltCo23)
        time.sleep(.25)

    # heading, roll, pitch = bno.read_euler()
    # #rc.ForwardMixed(address, 0)
    # time.sleep(.05)
    # #print heading
    # # if heading < 90:
    # #     heading = heading+360
    # #goal = heading-90
    # ser0.write(b'Z\r\n')
    # resp0 = (ser0.read(10))
    # resp0 = resp0[:8]
    # fltCo20 = float(resp0[2:])
    # ser1.write(b'Z\r\n')
    # resp1 = ser1.read(10)
    # resp1 = resp1[:8]
    # fltCo21 = float(resp1[2:])
    # ser2.write(b'Z\r\n')
    # resp2 = ser2.read(10)
    # resp2 = resp2[:8]
    # fltCo22 = float(resp2[2:])
    # ser3.write(b'Z\r\n')
    # resp3 = ser3.read(10)
    # resp3 = resp3[:8]
    # fltCo23 = float(resp3[2:])
    # C02Angle(fltCo20,fltCo21,fltCo22,fltCo23)
#    print goal


    rc.ForwardMixed(address, 0)
    rc.TurnRightMixed(address, 0)
