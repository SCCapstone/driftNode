"""
Silas Rubinson
CSCE 574
2/26/2015

using python 3.4 64-bit anaconda distribution for windows
I make no guarantee that this program will work with another version of python
"""

class Particle (object):
    """
    this is our particle object that will have all of the particle data such as x, y, theta and weight
    """

    def __init__ (self, x, y, theta, weight):
        """
        initilizing the values
        """

        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    def __repr__ (self):
        """
        return a string containing a printable representation of an object
        """
        return "(%f, %f, %f, %f)" % (self.x, self.y, self.theta, self.weight)

    @property
    def xy (self):
        """
        returns the x and y cordnates
        """
        return self.x, self.y

    @property
    def xyTheta (self):
        """
        returns the x, y, and theta values
        """
        return self.x, self.y, self.theta

    def incrementValues (self, x, y, theta):
        """
        increments the values x, y, and theta by the values passed in
        """
        self.x += x
        self.y += y
        self.theta += theta
