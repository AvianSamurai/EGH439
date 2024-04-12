##Setup
import matplotlib.pyplot as plt
import numpy as np

##get robot pose
RobX, RobY, = 0.2, 0.4

##Rearrange the equation in the form ax + by + c = 0
a = -1
b = 1
c = 0.001
x = np.linspace(0, 2, 1000)
y = (-a * x - c) / b


##plot line
fig1 = plt.figure()
plt.axis([0, 2, 0, 2])
plt.plot(x, y, linestyle = '-')

##plot robto
plt.plot(RobX,RobY,'ro') 
#plt.show

##find closest point
ClosestX = (b * (b * RobX - a * RobY) - a * c) / (a**2 + b**2)
ClosestY = (a * (-b * RobX + a * RobY) - b * c) / (a**2 + b**2)
ClosestPoint = ClosestX, ClosestY

plt.plot(ClosestX, ClosestY,'go') 
#plt.show()

##find end point
if ClosestX < 1:
  EndPoint = 1.8, ((-a * 1.8 - c) / b)
else:
  EndPoint = 0.2, ((-a * 0.2 - c) / b)

plt.plot(EndPoint[0], EndPoint[1],'bo') 
#plt.show()

PathX = np.linspace(ClosestX, EndPoint[0], 12)
PathY = np.linspace(ClosestY, EndPoint[1], 12)
PathX = PathX[1:11]
PathY = PathY[1:11]

plt.plot(PathX, PathY,'o') 
plt.show()