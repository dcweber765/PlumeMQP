import math
import numpy
from numpy import matrix
from numpy import linalg


def C02Angle(fltCo20,fltCo21,fltCo22,fltCo23):
    #Find the angle between 2 highest sensors see "Code Diagram.png"
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


	i = 1
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
def main();
    print("Running")
    fltCo20 = 460
    fltCo21 = 440
    fltCo22 = 450
    fltCo23 = 400
    print(C02Angle(fltCo20,fltCo21,fltCo22,fltCo23))
    print("End")

if __name__ =='__main__':
    main()
