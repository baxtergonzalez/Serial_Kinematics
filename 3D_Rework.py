from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import *
import numpy as np
import math
from math import cos, sin, sinh, tan, atan


fig = plt.figure()
ax = plt.gca(projection = '3d')
ax.set_xlim([-5,5])
ax.set_ylim([-5,5])
ax.set_zlim([0,10])

ax.set_xlabel('X Coords')
ax.set_ylabel('Y Coords')
ax.set_zlabel('Z Coords')

origin = np.array([0,0,0])

class TestLink:
    def __init__(self, start, displacement):
        self.start = np.array([start[0], start[1], start[2]])
        self.displacement = np.array([displacement[0], displacement[1], displacement[2]])
        self.end = self.start + self.displacement
        self.length = np.sum(np.square(self.displacement))


        print(f"I stretch from {self.start} to {self.end} with length {self.length}")
    def display(self):
        ax.quiver(self.start[0], self.start[1], self.start[2], self.end[0], self.end[1], self.end[2], color = 'blue')
class Target:
    def __init__(self):
        print("Type in new 3D coordinates...")
        self.x = int(input("X: "))
        self.y = int(input("Y: "))
        self.z = int(input("Z: "))
        self.coords = np.array([self.x, self.y, self.z])

        self.originDistance = np.sqrt(np.sum(np.square(self.coords, origin)))
        self.xyOriginDistance = ((self.x)**2 + (self.y)**2)**0.5
        self.thetaXOffset = np.degrees(math.atan(self.y/self.x))

        print(f"Coordinates are, X: {str(self.x)} Y:  {str(self.y)} Z: {self.z}") 
        self.display()

    def display(self):
        ax.scatter(self.coords[0], self.coords[1], self.coords[2])
        ax.text(self.coords[0] + 0.5, self.coords[1] + 0.5, self.coords[2], "Target")

class Link:
    def __init__(self, length, start, number = 0, color = "blue", debug = False):
        self.length = length
        self.start = start
        self.end = self.start + np.array([0,0, self.length]) #Initialize pointing straight up
        self.dispVector = self.end - self.start
        self.number = number
        self.color = color
        self.debug = debug
    def display(self):
        if self.debug:
            print(f"Start: {self.start}, End: {self.end}")
        ax.plot3D((self.start[0], self.end[0]), (self.start[1], self.end[1]), (self.start[2], self.end[2]), color = self.color)
    def rotateLink(self, eulerAngles):
        #X: +90, parallel with y axis in negative direction
        #Y: +90, parallel with x axis in positive direction
        #Z: +90, rotates about z axis, end position seems to be the same when starting pointing up

        eAngles = np.array(eulerAngles)

        if self.debug:
            print(f"Rotation Angles: {eAngles}")

        if self.debug:
            print(f"Initial Vector: {self.dispVector}")

        rot = R.from_euler('xyz', eAngles, degrees = True)
        rotationMatrix3D = np.array(rot.as_matrix())
        rotatedDispVector = np.dot(rotationMatrix3D, self.dispVector)
        newEnd = self.start + rotatedDispVector
        self.dispVector = rotatedDispVector

        if self.debug:
            print(f"Rotated Vector: {self.dispVector}")

        self.end = newEnd

        if self.debug:
            print(f"New End: {self.end}")

class Arm:
    def __init__(self, numLinks, target, startPt = np.array([0,0,0]), linkLength = 2):
        print(startPt)
        self.maxRange = 0
        self.target = target
        self.numLinks = numLinks
        self.startPt = startPt
        self.linkLength = linkLength
        self.colorOptions = ["midnightblue", "navy", "blue", "slateblue", "rebeccapurple", "blueviolet"]
        self.makeLinks()
    def makeLinks(self):
        self.links = []
        for i in range(0, self.numLinks):
            print(f"Making Link: {i}...")
            if i==0:
                #For the first link created, it starts at the origin of the arm
                self.manipulatorPosition = self.startPt
                self.links.append(Link(length = self.linkLength, start = self.manipulatorPosition, number = i, color = self.colorOptions[i], debug = True))
                self.maxRange += self.links[i].length
            elif i>0:
                #For all following links, their start is the end point of the link before it
                self.manipulatorPosition = self.links[i-1].end
                self.links.append(Link(length = self.linkLength, start = self.manipulatorPosition, number = i, color = self.colorOptions[i], debug = True))
                
                self.maxRange += self.links[i].length
                self.manipulatorPosition = self.links[i].end
            #self.links[i].display()
        print(f"Original Manipulator Position: {self.manipulatorPosition}\n")
    def updateLinks(self, alteredLink, rotationAngles):
        for i in range(alteredLink.number, self.numLinks):
            #For all links past the link that has been rotated/altered
            if i < self.numLinks-1:
                #For all links from the altered link up to the penultimate link...
                print(f"Updating Link {i}...")
                self.links[i].rotateLink(rotationAngles)
                self.links[i+1].start = self.links[i].end
                #Calculate a new ending point from the angle and 
            else:
                #For the last link in the arm
                print(f"Updating Link {i}...")
                self.links[i].rotateLink(rotationAngles)
                self.manipulatorPosition = self.links[i].end
                print(f"Manipulator Position is {self.manipulatorPosition}")
            #self.links[i].display()
    # def findSquaredManipDistance(self):
    #     point2 = self.target.coords
    #     point1 = self.manipulatorPosition
    #     distance = np.sum(np.square(point2-point1))
    #     return distance
    def findSquaredPlanarManipDistance(self):
        point2 = np.array([self.target.xyOriginDistance, self.target.coords[2]]) #radius offset and z value
        point1 = np.array([self.manipulatorPosition[0], self.manipulatorPosition[2]])
        distance = np.sum(np.square(point2 - point1))
        return distance

    def chaseTarget(self, deltaTheta = [0,5, 0], deltaThetaNeg = [0, -5, 0], threshold = 0.1, changeThresh = 1):
        if self.target.originDistance < self.maxRange:
            while self.findSquaredPlanarManipDistance() > threshold: #TODO: CHANGE THIS THRESHOLD VALUE, SEEMS WACK
                #for as long as the manipulator is far enough away from the target...
                for i in range(len(self.links)-1, -1, -1):
                    #iterate through the links starting with the end link...
                    d1 = self.findSquaredPlanarManipDistance()
                    self.updateLinks(self.links[i], deltaTheta)
                    d2 = self.findSquaredPlanarManipDistance()
                    print(f"D1: {d1}, D2: {d2}")
                    if d2 < d1:
                        print("I want to rotate cCW")
                        #if all of the links rotated 'deltaTheta' degrees to the left gets the manipulator closer...
                        while d2 < d1:
                            print("\nRotating cCW...")
                            #Continue to rotate them in that direction until goin that direction takes the manipulator further away
                            self.updateLinks(self.links[i], deltaTheta)
                            d1 = d2
                            d2 = self.findSquaredPlanarManipDistance()
                            print(f"D1: {d1}, D2: {d2}")
                        self.updateLinks(self.links[i], deltaThetaNeg)
                        #bc the last step brought manipulator further away from target, undo that last step
                    elif d2 > d1:
                        print("I want to rotate CW")
                        #same as above, but rotate to the right
                        self.updateLinks(self.links[i], deltaThetaNeg)
                        self.updateLinks(self.links[i], deltaThetaNeg)
                        d2 = self.findSquaredPlanarManipDistance()
                        while d2 < d1:
                            print("\nRotating CW")
                            d1 = d2
                            self.updateLinks(self.links[i], deltaThetaNeg)
                            d2 = self.findSquaredPlanarManipDistance()
                            print(f"D1: {d1}, D2: {d2}")
                        self.updateLinks(self.links[i], deltaTheta)
            #Rotate the whole arm about he z axis to grab the target!
            print(f"Final Planar Distance: {np.sqrt(d1)}")
            print(self.target.thetaXOffset)
            self.updateLinks(self.links[0], np.array([0,0,self.target.thetaXOffset]))
        else:
            print("Out of Range :( ")

    def display(self):
        for link in self.links:
            link.display()


myTarget = Target()
myTarget.display()

myArm = Arm(numLinks = 5, target = myTarget, linkLength = 2)
myArm.chaseTarget()
myArm.display()

plt.show()