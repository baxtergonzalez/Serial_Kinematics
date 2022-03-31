#TURN ON RUN IN TERMINAL SETTING

import matplotlib.pyplot as plt
import numpy as np
import math

plt.plot(0,0,'ok')
plt.axis('equal')
plt.xlim(-5, 5)
plt.ylim(0,5)
plt.grid(visible=True, which='major')

class Target:
    def __init__(self):
        print("Type in new target Coordinates...\n")
        self.x = int(input("X: "))
        self.y = int(input("Y: "))
        self.coords = np.array([self.x, self.y])
        self.originDistance = np.sqrt(np.sum(np.square(self.coords, np.array([0,0]))))
        print("Coordinates are, X: " + str(self.x) + " Y: " + str(self.y)) 
        plt.plot(self.x, self.y, marker = "o", markersize = 15, markeredgecolor = "red", markerfacecolor = "red")
        plt.text(self.x + 0.5, self.y, "Target")

    def display(self):
        plt.plot(self.x, self.y, marker = "o", markersize = 15, markeredgecolor = "red", markerfacecolor = "red")
        plt.text(self.x + 0.5, self.y, "Target")
    def update(self):
        self.x = int(input("New X: "))
        self.y = int(input("New Y: "))
        self.coords = np.array([self.x, self.y])
        print("Coordinates are, X: " + str(self.x) + " Y: " + str(self.y)) 
        plt.plot(self.x, self.y, marker = "o", markersize = 15, markeredgecolor = "red", markerfacecolor = "red")
        plt.text(self.x + 0.5, self.y, "Target")

class Link:
    def __init__(self, length, start, angle, number =0, color = "blue"):
        self.length = length
        self.start = np.array(start)
        self.angle = angle
        self.end = self.findEnd(self.angle)
        self.number = number
        self.color = color
        # print(f"My initial start is: {self.start}")
        # print(f"My initial end is: {self.end}")
    def display(self):
        plt.plot([self.start[0], self.end[0]], [self.start[1], self.end[1]], color = self.color)
    def findEnd(self, angleDeg):
        angle = math.radians(angleDeg)
        end = np.array([self.start[0] + self.length*np.cos(angle), self.start[1] + self.length*np.sin(angle)])
        self.dispVect = end - self.start
        return end
    def rotateLink(self, deltaTheta):
        print(f"Rotating Link {self.number}...")
        theta = math.radians(deltaTheta)
        rotationMatrix2D = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        newEnd = self.start + np.dot(rotationMatrix2D, self.dispVect)
        self.dispVect = newEnd - self.start
        self.end = newEnd
        return newEnd
    def findDistance(self, point1, point2):
        distance = np.sqrt(np.sum(np.square(point2-point1)))
        return distance
    def chasePoint(self, target):
        self.angle = self.findMinDistance(target)
        self.end = self.findEnd(self.angle)
        print("My new End point is: "+ str(self.end))

class Arm:
    def __init__(self, numLinks, target, startPt = np.array([0,0]), linkLength = 2):
        self.maxRange = 0
        self.target = target
        self.numLinks = numLinks
        self.startPt = startPt
        self.linkLength = linkLength
        self.colorOptions = ["midnightblue", "navy", "blue", "slateblue", "rebeccapurple", "blueviolet"]
        self.makeLinks()
    
    def findSquaredManipDistance(self):
        #Finds the squared distance from the manipulator to the target (saves on processing to not find sqrt)
        point2 = self.target.coords
        point1 = self.manipulatorPosition
        distance = np.sum(np.square(point2-point1))
        return distance 
    def findManipDistance(self):
        point2 = self.target.coords
        point1 = self.manipulatorPosition
        distance = np.sqrt(np.sum(np.square(point2-point1)))
        return distance
    def makeLinks(self):
        #Create a list of link objects of quantity 'numLinks'
        self.links = []
        for i in range(0, self.numLinks):
            print(f"Link: {i}")
            if i==0:
                #For the first link created, it starts at the origin of the arm
                self.manipulatorPosition = self.startPt
                self.links.append(Link(length = self.linkLength, start = self.manipulatorPosition, number = i, angle = 90, color = self.colorOptions[i]))
                self.maxRange += self.links[i].length
            elif i>0:
                #For all following links, their start is the end point of the link before it
                self.manipulatorPosition = self.links[i-1].end
                # print(f"Manipulator Position: {self.manipulatorPosition}")
                self.links.append(Link(length = self.linkLength, start = self.manipulatorPosition, number = i, angle = 90, color = self.colorOptions[i]))
                #All links start with oriented 90 degrees (straight up)
                self.maxRange += self.links[i].length
                self.manipulatorPosition = self.links[i].end
            self.links[i].display()
        print(f"Original Manipulator Position: {self.manipulatorPosition}\n")
    def updateLinks(self, alteredLink, rotationAngle):
        for i in range(alteredLink.number, self.numLinks):
            #For all links past the link that has been rotated/altered
            if i < self.numLinks-1:
                #For all links from the altered link up to the penultimate link...
                print(f"Updating Link {i}...")
                self.links[i].rotateLink(rotationAngle)
                self.links[i+1].start = self.links[i].end
                #Calculate a new ending point from the angle and 
            else:
                #For the last link in the arm
                print(f"Updating Link {i}...")
                self.links[i].rotateLink(rotationAngle)
                self.manipulatorPosition = self.links[i].end
                print(f"Manipulator Position is {self.manipulatorPosition}")
    def chaseTarget(self, deltaTheta = 1, threshold = np.sqrt(0.2)):
        if self.target.originDistance < self.maxRange:
            while self.findSquaredManipDistance() > threshold:
                #for as long as the manipulator is far enough away from the target...
                for i in range(len(self.links)-1, -1, -1):
                    # print(f"Rotating Link {i}...")
                    #iterate through the links starting with the end link...
                    d1 = self.findSquaredManipDistance()
                    self.updateLinks(self.links[i], deltaTheta)
                    d2 = self.findSquaredManipDistance()
                    print(f"D1: {d1}, D2: {d2}")
                    if d2 < d1:
                        print("I want to rotate cCW")
                        #if all of the links rotated 'deltaTheta' degrees to the left gets the manipulator closer...
                        while d2 < d1:
                            print("\nRotating cCW...")
                            #Continue to rotate them in that direction until goin that direction takes the manipulator further away
                            self.updateLinks(self.links[i], deltaTheta)
                            d1 = d2
                            d2 = self.findSquaredManipDistance()
                            print(f"D1: {d1}, D2: {d2}")
                        self.updateLinks(self.links[i], -deltaTheta)
                        #bc the last step brought manipulator further away from target, undo that last step
                    elif d2 > d1:
                        print("I want to rotate CW")
                        #same as above, but rotate to the right
                        self.updateLinks(self.links[i], -deltaTheta)
                        self.updateLinks(self.links[i], -deltaTheta)
                        d2 = self.findSquaredManipDistance()
                        while d2 < d1:
                            print("\nRotating CW")
                            d1 = d2
                            self.updateLinks(self.links[i], -deltaTheta)
                            d2 = self.findSquaredManipDistance()
                            print(f"D1: {d1}, D2: {d2}")
                        self.updateLinks(self.links[i], deltaTheta)
        else:
            print("Out of Range :( ")
            
    def displayArm(self):
        plt.clf()
        plt.plot(0,0,'ok')
        plt.axis('equal')
        plt.xlim(-5, 5)
        plt.ylim(0,5)
        plt.grid(visible=True, which='major')

        self.target.display()
        for link in self.links:
            link.display()



myTarget = Target()
#link1 = Link(length = 2, start = np.array([0,0]), angle = 90)


myArm = Arm(numLinks = int(input("NumLinks: ")), target = myTarget, linkLength = 1)
myArm.chaseTarget()
myArm.displayArm()

#link1.chasePoint(myTarget)
#link1.display()
plt.show()
