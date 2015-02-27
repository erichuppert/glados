#!/usr/bin/python

from numpy import polyfit as pfit
import matplotlib.pyplot as plt

deg = 1 # We're fitting a linear equation

distances = [float(d) for d in input("Insert distances in meters, separated by spaces: ").split(' ')]
areas = [float(a) for a in input("Insert measured areas in pixels, separated by spaces: ").split(' ')]

y = distances2 = [d**2 for d in distances]
x = areas_inv = [1/a for a in areas]

plt.ylabel("(Distance)^2")
plt.xlabel("1/(Area)")

a,b = pfit(x,y,1)

plt.plot(x,y,'ro',)
plt.plot(x,[a*x+b for x in x])

plt.legend(["Data", "d^2 = %f * 1/a + %f" % (a,b)])

plt.show()

print("(distance)^2 = %f * 1/(area) + %f" % (a,b))