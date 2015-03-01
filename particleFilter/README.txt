the program was tested to run with the anaconda distribution of python 3.4 64-bit

to use in windows use the command from the directory these files are
python particleFilter.py <number of particles you want> <number of steps to run> <behaviour type>

all of the arguments are numbers

behaviour are 
1 is fixed position (0,0) with random orientation
2 is fixed position (0,0) with random orientation noise added to velocities

3 is uniformly distributed over a 10m by 10m area with random orientation
4 is uniformly distributed over a 10m by 10m area with random orientation noise added to velocities

5 fixed x and y but with noise and theta = 90deg
6 fixed x and y but with noise and theta = 90deg noise added to velocities