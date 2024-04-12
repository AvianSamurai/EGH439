##Setup
import matplotlib.pyplot as plt
import numpy as np

##get robot pose
RobX, RobY, = 0.2, 0.4
Num = 25
NumSpec = 100

##generate path
t = np.linspace(0, 2*np.pi, Num)
tLine = np.linspace(0, 2*np.pi, NumSpec)
a = 0.75 / np.sqrt(2)

PathX = (a * np.sqrt(2) * np.cos(t)) / (np.sin(t)**2 + 1) + 1
PathY = (a * np.sqrt(2) * np.cos(t) * np.sin(t)) / (np.sin(t)**2 + 1) + 1

xLine = (a * np.sqrt(2) * np.cos(tLine)) / (np.sin(tLine)**2 + 1) + 1
yLine = (a * np.sqrt(2) * np.cos(tLine) * np.sin(tLine)) / (np.sin(tLine)**2 + 1) + 1

print(PathX)
print(PathY)

##plot path
fig1 = plt.figure()
plt.axis([0, 2, 0, 2])
plt.plot(PathX, PathY, 'bo')
plt.plot(xLine, yLine, 'g-')
#plt.show()

##plot robto
plt.plot(RobX,RobY,'ro') 
plt.show()

##find closest point
i = 0
j = 0
CloPeak = 100
while i < NumSpec:
    CloMeas = np.sqrt((RobX - xLine[i])**2 + (RobY - yLine[i])**2)
    if CloPeak > CloMeas:
        j = i
        CloPeak = CloMeas
    i = i + 1
ClosestPoint = xLine[j], yLine[j]

#plt.plot(ClosestPoint[0], ClosestPoint[1],'go') 
#plt.show()

PathX[:0] = [ClosestPoint[0]]
PathY[:0] = [ClosestPoint[1]]

plt.axis([0, 2, 0, 2])
plt.plot(PathX, PathY,'bo') 
plt.plot(ClosestPoint[0], ClosestPoint[1],'go') 
plt.plot(RobX, RobY,'ro') 
plt.show()