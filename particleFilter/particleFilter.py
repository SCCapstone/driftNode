"""
Silas Rubinson
CSCE 574
2/26/2015

using python 3.4 64-bit anaconda distribution for windows
I make no guarantee that this program will work with another version of python
"""

import math
import matplotlib.pyplot as plt
import random
import sys
from particle import Particle

#making sure the user knows what should be input
if len(sys.argv) != 4:
    print("useage <number of particles> <how steps to run> <behavior from 1 to 6>")
    sys.exit()

#getting the input paramiters
numberOfparticles = int(sys.argv[1])
numberOfSteps = int(sys.argv[2])

"""
1 is fixed position (0,0) with random orientation
2 is fixed position (0,0) with random orientation noise added to velocities

3 is uniformly distributed over a 10m by 10m area with random orientation
4 is uniformly distributed over a 10m by 10m area with random orientation noise added to velocities

5 fixed x and y but with noise and theta = 90deg
6 fixed x and y but with noise and theta = 90deg noise added to velocitites
"""
behaviorToSelect = int(sys.argv[3])

#the empty list that will hold the particles after we fill it
particleList = []


#initilizing the particles

#behavior 1 or 2 all particles at a fixed x any y with random orientaion
if (behaviorToSelect == 1) or (behaviorToSelect == 2):
    for i in range(0, numberOfparticles):
        particleList.append(Particle(0.0, 0.0, math.radians(random.randint(0, 360)), 1 / numberOfparticles))

#behavior 3 or 4 random x, y, and theta
elif (behaviorToSelect == 3) or (behaviorToSelect == 4):
    for i in range(0, numberOfparticles):
        particleList.append(Particle(random.uniform(-5, 5), random.uniform(-5, 5), math.radians(random.randint(0, 360)), 1 / numberOfparticles))

#behavior 5 or 6 fixed x and y but with noise and theta = 90deg
elif (behaviorToSelect == 5) or (behaviorToSelect == 6):
    for i in range(0, numberOfparticles):
        particleList.append(Particle(random.normalvariate(0.0, 0.3), random.normalvariate(0.0, 0.3), math.radians(90), 1 / numberOfparticles))

#odd numbers are fixed velocities v=2m/s a=0.1rad/s
if ((behaviorToSelect) % 2) != 0:
    velocity = 2.0
    angularVelocity = 0.1
    noise = False

#even numbers are fixed velocity v=2m/s random angular velocity (-0.2, 0.2)rad/s updating every
#10 steps with noise added to the velocity and angular velocity
elif (behaviorToSelect % 2) == 0:
    velocity = 2.0
    angularVelocity = random.uniform(-0.2, 0.2)
    noise = True

#the loop that will update the particles position
for timeStep in range(0, numberOfSteps):

    #updating the angular velocity if we are using the second motion model
    if (noise == True) and ((timeStep + 1) % 10 == 0):
        angularVelocity = random.uniform(-0.2, 0.2)

    #updating the particles position in the world with no noise
    if noise == False:

        #looping over all of the particles
        for ithParticle in particleList:

            #updating the values of the particles
            ithParticle.x = ithParticle.x + (velocity * math.cos(ithParticle.theta))
            ithParticle.y = ithParticle.y + (velocity * math.sin(ithParticle.theta))
            ithParticle.theta = ithParticle.theta + angularVelocity

    #updating the particles position in the world with noise
    if noise == True:

        #looping over all of the particles
        for ithParticle in particleList:

            #updating the values of the particles and adding noise
            ithParticle.y = ithParticle.y + ((velocity + random.normalvariate(0.0, 0.1)) * math.sin(ithParticle.theta))
            ithParticle.x = ithParticle.x + ((velocity + random.normalvariate(0.0, 0.1)) * math.cos(ithParticle.theta))
            ithParticle.theta = ithParticle.theta + angularVelocity + random.normalvariate(0.0, 0.001)

#plotting the positions of the particles
x = []
y = []
for ithParticle in particleList:
    x.append(ithParticle.x)
    y.append(ithParticle.y)

plt.scatter(x, y,)
plt.show()
