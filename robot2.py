import time
from roboclaw import Roboclaw
import csv
import logging
import sys
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

rc = Roboclaw("/dev/ttyACM0",115200)

rc.Open()
address = 0x80

startTime = time.strftime("%Y-%m-%d_%H:%M:%S")


def startup():
    #CO2 startup
    ser0.flushInput()
    ser1.flushInput()
    ser2.flushInput()
    ser3.flushInput()
    time.sleep(1)
    ser0.write(b'G\r\n') #Calibrate CO2 sensors to ~350 ppm
    ser1.write(b'G\r\n')
    ser2.write(b'G\r\n')
    ser3.write(b'G\r\n')
    print('Zero Point Cal Air')
    time.sleep(1)


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

def lowpassfilter(Z, Z_filt):
    Z_filt = (.3758*Z)+(.6242*Z_filt)
    return Z_filt



def turnLeft(speed, TurnAng):
    #Turn Left using PWM speed control and IMU data
    heading, roll, pitch = bno.read_euler()
    CurrentHeading = heading
    goal = TurnAng + CurrentHeading
    if goal > 360:
        goal = goal - 360
    print "left", goal
    while not(goal-5 <= heading <= goal+5): #+
        heading, roll, pitch = bno.read_euler()
        rc.TurnRightMixed(address, speed)
    rc.TurnRightMixed(address,0)
    time.sleep(.1)

def turnRight(speed, TurnAng):
    #Turn Right using PWM speed control and IMU data
    heading, roll, pitch = bno.read_euler()
    CurrentHeading = heading
    goal = TurnAng + CurrentHeading
    if goal > 360:
        goal = goal - 360
    print "Right", goal
    while not(goal-5 <= heading <= goal+5): #-
        heading, roll, pitch = bno.read_euler()
        rc.TurnLeftMixed(address, speed)
    rc.TurnLeftMixed(address, 0)
    time.sleep(.1)

def C02Angle(Co2_0,Co2_1,Co2_2,Co2_3):
    #Find the angle between 2 highest sensors see "Code Diagram.png"
    D = 15.5
    c = 11.25
    B = math.pi/4

    inital = [Co2_0,Co2_1,Co2_2,Co2_3]

    i = 0
    sortedvalues = sorted(inital, reverse=True)

    print sortedvalues
    # if sortedvalues[i] == Co2_0:
    #     print "Case 1"
    #
    # if sortedvalues[i] == Co2_1:    # turns right 90*
    #     turnLeft(32, 360)
    #     print "Case 2"
    #
    # if sortedvalues[i] == Co2_2:   #Backwards
    #     turnRight(32, 180)
    #     print "Case 3"
    #
    # if sortedvalues[i] == Co2_3:  #for turning right
    #     turnRight(32, 90)
    #     print "Case 4"

    ###### Angles
    if sortedvalues[i] == Co2_0 and sortedvalues[i+1] == Co2_3:
        goal = angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnRight(32,goal)

    if sortedvalues[i] == Co2_1 and sortedvalues[i+1] == Co2_0:
        goal = 90-angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnLeft(32,goal)

    if sortedvalues[i] == Co2_1 and sortedvalues[i+1] == Co2_2:
        goal = 90+angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnLeft(32,goal)

    if sortedvalues[i] == Co2_2 and sortedvalues[i+1] == Co2_1:
        goal = 180-angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnLeft(32,goal)

    if sortedvalues[i] == Co2_2 and sortedvalues[i+1] == Co2_3:
        goal = 180-angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnRight(32,goal)


    if sortedvalues[i] == Co2_3 and sortedvalues[i+1] == Co2_0:
        goal = 90-angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnRight(32,goal)

    if sortedvalues[i] == Co2_3 and sortedvalues[i+1] == Co2_2:
        goal = 90+angle(sortedvalues[0]-sortedvalues[2],sortedvalues[1]-sortedvalues[2])
        turnRight(32,goal)

    else:
        a = 0
        b = math.sqrt(a**2+c**2-(2*a*c*math.cos(B)))
        A = math.acos((-a**2+b**2+c**2)/(2*b*c))

        AngDeg = A*(180/math.pi)
        return AngDeg


def angle(sensor1,sensor2):

    i = 0
    D = 15.5
    c = 11.25
    print(sensor1)
    print(sensor2)
    B = (math.pi)/4
    a = sensor2*D / (sensor2 + sensor1)

    b =math.sqrt(a**2+c**2-(2*a*c*math.cos(B)))
    A = math.acos((-a**2+b**2+c**2)/(2*b*c))
    print(A*(180/math.pi))

    return A*(180/math.pi)

def displayspeed():
    #Print positon and velocity data from roboClaw
    #function is given from roboclaw examples
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

def getMotorSpeed():
    speed1 = rc.ReadSpeedM1(address)
    speed2 = rc.ReadSpeedM2(address)

    return speed1[1], speed2[1]


def getCO2Data():
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

    Co2_0 = fltCo20 * 10
    Co2_1 = fltCo21 * 10
    Co2_2 = fltCo22 * 10
    Co2_3 = fltCo23 * 10

    return Co2_0, Co2_1, Co2_2, Co2_3

def printSensorData():
    #Print sensor data to consol
    heading, roll, pitch = bno.read_euler()
    accelX, accelY, accelZ  = bno.read_accelerometer()
    gyroX, gyroY, gyroZ = bno.read_gyroscope()
    LinAccelX, LinAccelY, LinAccelZ = bno.read_linear_acceleration()
    Co2_0, Co2_1, Co2_2, Co2_3 = getCO2Data()
    displayspeed()
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}'.format(heading, roll, pitch))
    print('accelX={0:0.2F} accelY={1:0.2F} accelZ={2:0.2F}'.format(accelX, accelY, accelZ))
    print 'CO2 PPM   =', Co2_0
    print 'CO2 PPM 2 =', Co2_1
    print 'CO2 PPM 3 =', Co2_2
    print 'CO2 PPM 4 =', Co2_3

def driveForward(inch):
    leftSpeeds = 0
    rightSpeeds = 0
    iter = 0
    rc.ResetEncoders(address)

    rc.SpeedAccelDistanceM1(address,3000,3000,421*inch,1);
    rc.SpeedAccelDistanceM2(address,3000,3000,421*inch,1);
    buffers = (0,0,0)
    while(buffers[1]!=0x80 and buffers[2]!=0x80):   #Loop until distance command has completed
        speed1, speed2 = getMotorSpeed()

        leftSpeeds = (speed1) + (leftSpeeds)
        rightSpeeds = (speed2) + (rightSpeeds)
        iter = iter+1
        buffers = rc.ReadBuffers(address)

    avgLeftSpeed = leftSpeeds/iter
    avgRightSpeed = rightSpeeds/iter

    avgLeftSpeed = avgLeftSpeed*((math.pi*0.075565)/(1253)) #m/sec
    avgRightSpeed = avgRightSpeed*((math.pi*0.075565)/(1253)) #m/sec
    time.sleep(.1)

    return avgLeftSpeed, avgRightSpeed

def endCondtion():
    condition  = False
    maxCo2 = 5000 #Find saturated sensor value
    Co2_0, Co2_1, Co2_2, Co2_3 = getCO2Data()
    maxed_0 = Co2_0 > maxCo2
    maxed_1 = Co2_1 > maxCo2
    maxed_2 = Co2_2 > maxCo2
    maxed_3 = Co2_3 > maxCo2
    saturated = maxed_0 and maxed_1 and maxed_2 and maxed_3
    if(saturated):
        condition = True

    return condition


def main():
    startup()

    print "waiting"
    time.sleep(10)
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
            printSensorData()
            time.sleep(.1)
            Co2_0, Co2_1, Co2_2, Co2_3 = getCO2Data()
            C02Angle(Co2_0, Co2_1, Co2_2, Co2_3)
            vl, vr = driveForward(1)
            print vl, vr
            time.sleep(.5)
            accelX, accelY, accelZ  = bno.read_accelerometer()
            gyroX, gyroY, gyroZ = bno.read_gyroscope()
            Z = matrix([[float(accelX)], [float(accelY)], [float(gyroZ)]])
            ts = .05
            xhat, P = ekf(vl, vr, Z, ts, x_i, y_i, theta_i, P)
            x_i = xhat.item(0) #position in meters
            y_i = xhat.item(1) #position in meters
            theta_i = xhat.item(2) #angle in rads?
            print xhat
            writer.writerow({'X' : x_i, 'Y' : y_i, 'Theta' : theta_i})
    print "Done!"
    print xhat




if __name__ == "__main__":
    main()
