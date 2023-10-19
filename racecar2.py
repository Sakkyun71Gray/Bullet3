import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math

physicsClient = p.connect(p.DIRECT) 

#直進
p.resetSimulation()
#p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
timestep = 1. / 240.
p.setTimeStep(timestep)
#momo_envId = p.loadURDF("sampleM.urdf", [0,0,-1])
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [3,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,3.14])

StartPos2 = [-1,0,0] 
StartOrient2 = p.getQuaternionFromEuler([0,0,- math.pi / 2]) 

carId = p.loadURDF("/data/racecar/racecar.urdf",cubeStartPos, cubeStartOrientation)
sample_carId = p.loadURDF("/sample/urdf/urdf_sample.urdf", StartPos2, StartOrient2)

frame = []
frame2 = []
frame3 = []
Positions = []
for t in range (640):
    #p.setJointMotorControl2(carId, 2, p.VELOCITY_CONTROL, targetVelocity=10)
    #p.setJointMotorControl2(carId, 3, p.VELOCITY_CONTROL, targetVelocity=10)
    #p.setJointMotorControl2(carId, 5, p.VELOCITY_CONTROL, targetVelocity=10)
    #p.setJointMotorControl2(carId, 7, p.VELOCITY_CONTROL, targetVelocity=10)
    p.setJointMotorControl2(sample_carId, 0, p.VELOCITY_CONTROL, targetVelocity=10)
    p.setJointMotorControl2(sample_carId, 1, p.VELOCITY_CONTROL, targetVelocity=10)
    p.stepSimulation()
    if t % 8 == 0:
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(360,240)
        frame.append(rgbImg)
        #frame2.append(depthImg)
        #frame3.append(segImg)
        Position, Orientation = p.getBasePositionAndOrientation(carId)
        Positions.append(Position)

images =[]
for im in frame:
  img = Image.fromarray(im)
  images.append(img)
images[0].save('Racecar_moving2.gif',save_all=True, append_images=images[:], optimize=False, duration=40, loop=0)

times = list(range(80))

Positions = np.array(Positions).T.tolist()
#print(Positions[0])

"""
plt.plot(times, Positions[0])
plt.plot(times, Positions[1])
plt.plot(times, Positions[2])
"""
#plt.plot(Positions[0], Positions[1])
plt.plot(times, Positions[0])
plt.plot(times, Positions[1])
plt.plot(times, Positions[2])
plt.show()


"""
images2 =[]
for im in frame2:
  img = Image.fromarray(im)
  images2.append(img)
images2[0].save('Racecar_moving3.gif',save_all=True, append_images=images[:], optimize=False, duration=40, loop=0)

images3 =[]
for im in frame3:
  img = Image.fromarray(im)
  images3.append(img)
images3[0].save('Racecar_moving4.gif',save_all=True, append_images=images[:], optimize=False, duration=40, loop=0)
"""
