#TURN ON RUN IN TERMINAL SETTING

import matplotlib.pyplot as plt
import numpy as np

plt.plot(0,0,'ok')
plt.axis('equal')
plt.xlim(-5, 5)
plt.ylim(0,5)
plt.grid(b=True, which='major')

class Target:
    def __init__(self):
        self.update()
        self.coords = [0,0]
        self.txt = "Hello"
    def update(self):
        print("Type in new target Coordinates...\n")
        self.x = int(input("X: "))
        self.y = int(input("Y: "))
        print("Coordinates are, X: " + str(self.x) + " Y: " + str(self.y)) 
        plt.plot(self.x, self.y, 'ok')

class Link:
    def __init__(self, length, start, angle):
        self.length = np.array(length)
        self.start = np.array(start)
        self.angle = angle
        self.findEnd(self.angle)
    def display(self):
        plt.plot(self.start, self.end)
    def findEnd(self, angle):
        end = self.start + np.array([self.length*np.cos(angle), self.length*np.sin(angle)])
        return end
    def findDistance(self, point1, point2):
        distance = np.sqrt((point2[0]-point1[0])**2 + (point2[1]-point2[1])**2)
        return distance
    def gradientDescent(self, target, deltaTheta = 2, slopeThreshold = 1/20):
        #gradient descent algorithm on graph of theta vs endpoint Distance To Target
        theta1 = self.angle
        p1 = self.findEnd(theta1)
        theta2 = self.angle + deltaTheta
        p2 = self.findEnd(theta2)
        #Find points in question, one at the original angle, other at incremented angle
        p1Distance = self.findDistance(p1, target)
        p2Distance = self.findDistance(p2, target)
        #Find the distance of those pts to target pt

        slope = (p2Distance - p1Distance)/deltaTheta

        while abs(slope)<slopeThreshold:
            #first look to see if the slope is negative, 
            if slope<0:
                #If slope is negative (IE distance is decreasing as angle increases)
                theta1 = theta2
                p1 = p2
                p1Distance = p2Distance
                theta2 = theta2 + deltaTheta
                #Set theta1 equal to theta2 because we know it is closer, then increment theta2 up
            else:
                #If slope is positive (IE distance is increasing as angle increases)
                theta1 = theta1
                p1 = p1
                p1Distance = p1Distance
                theta2 = theta1 - deltaTheta
                #Leave theta1 as is, and try decrementing theta2
            p2 = self.findEnd(theta2)
            p2Distance = self.findDistance(p2, target)
            #calculate new p2Distance (leave p1 because there is no case where it is not set in above logic)
            slope = (p2Distance - p1Distance)/deltaTheta
            #find slope with new points
        self.angle = theta1
        #Upon completing algorithm to find a local min, we set that as our new angle for the linkage
    def chasePoint(self, target):
        self.gradientDescent(target)
        self.end = self.findEnd(self.angle)
    

    
def drawVect(end, start = [0,0]):
        plt.plot([start[0], end.x], [start[1], end.y])
        plt.show()

myTarget = Target()
drawVect(myTarget)
