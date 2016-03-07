#! /usr/bin/python
################################################################
### This is a tool used to plot the output csv file from     ###
### the workspace_mapper node.                               ###
### author: Oliver Ebeling-Koning <odek@vt.edu>              ###
### date: 09/07/2015                                         ###
################################################################

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits import mplot3d
from stl import mesh
import csv, time
import numpy as np
import sys, os
import math




#################################
# Subsample to display
subsampleEach = 1
#################################



# Check arguments
if len(sys.argv) != 2:
	print "ERROR: wrong number of arguments.\n\tUSAGE: This tool takes in the csv file to use as an argument"
	exit(1)
if not os.path.exists(sys.argv[1]):
	print "ERROR: the file `%s` does not exist" % sys.argv[1]
	exit(1)
	
# Make lists to hold values
xList = []
yList = []
zList = []


counter = 10
# Read in csv File
with open(sys.argv[1], 'rb') as csvFile:
	csvReader = csv.reader(csvFile, delimiter=',')
	for row in csvReader:
		counter = counter + 1
		if counter % subsampleEach == 0:
			if not len(row) < 3:
				xList.append(float(row[0]))
				yList.append(float(row[1]))
				zList.append(float(row[2]))



# Get distances
distances = []
for ii in range(0, len(xList)):
	# dist = np.linalg.norm(a-b)
	distances.append(math.sqrt(math.pow(xList[ii],2)+math.pow(yList[ii],2)+math.pow(zList[ii],2)))
# Scale all distances to 0-100 for colormap (y=y1+((x-x1)(y2-y1))/(x2-x1))
maximumDist = max(distances) # y2 ... x2=100
minimumDist = min(distances) # y1 .. x1=0
newDistances = []
for jj in range(0, len(xList)):
	newDistances.append((minimumDist)+((distances[jj]*(maximumDist-minimumDist))/(100)))




fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



ax.scatter(xList, yList, zList, s=70, c=newDistances, cmap='Purples', alpha=0.2)
ax.set_xlabel('X Axis (meters)')
ax.set_ylabel('Y Axis (meters)')
ax.set_zlabel('Z Axis (meters)')
ax.set_title('ABB IRB1200 7/0.7 Reachability Plot')
ax.set_xlim3d(-0.4, 1.0)
ax.set_ylim3d(-0.7, 0.7)
ax.set_zlim3d(0.0, 1.4)

# Plot a print volume
xOffset = 0.4
yOffset = 0
zOffset = .4
xDist = .4
yDist = .4
zDist = .75
stepSize = 2
xVector = np.linspace(xOffset-(xDist/2.0), xOffset+(xDist/2.0), num=stepSize)
yVector = np.linspace(yOffset-(yDist/2.0), yOffset+(yDist/2.0), num=stepSize)
zVector = np.linspace(zOffset-(zDist/2.0), zOffset+(zDist/2.0), num=stepSize)
# 1) Side
#X, Y = np.meshgrid(xVector, yVector)
# Z = np.ones_like( X )
# Z = Z*(zOffset-(zDist/2.0))
# ax.plot_wireframe(X,Y,Z, color="red")
# 2) Side
# Z = np.ones_like( X )
# Z = Z*(zOffset+(zDist/2.0))
# ax.plot_wireframe(X,Y,Z, color="red")
# 3) Side
# Y, Z = np.meshgrid(yVector, zVector)
# X = np.ones_like( Y )
# X = X*(xOffset-(xDist/2.0))
# ax.plot_wireframe(X,Y,Z, color="red")
# 4) Side
# Y, Z = np.meshgrid(yVector, zVector)
# X = np.ones_like( Y )
# X = X*(xOffset+(xDist/2.0))
# ax.plot_wireframe(X,Y,Z, color="red")
# 5) Side
# X, Z = np.meshgrid(xVector, zVector)
# Y = np.ones_like( X )
# Y = Y*(yOffset-(yDist/2.0))
# ax.plot_wireframe(X,Y,Z, color="red")
# ) Side
# X, Z = np.meshgrid(xVector, zVector)
# Y = np.ones_like( X )
# Y = Y*(yOffset+(yDist/2.0))
# ax.plot_wireframe(X,Y,Z, color="red")







# Plot robot in 3d plot
robotMesh = mesh.Mesh.from_file("IRB1200_7_07.stl")
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(robotMesh.vectors))


plt.show()
