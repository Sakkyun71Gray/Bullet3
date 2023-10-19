import pybullet as p
import pybullet_data
import numpy as np
#import matplotlib.pyplot as plt
#from PIL import Image

from time import sleep

physicsClient = p.connect(p.GUI, "option=opengl2")
#physicsClient = p.connect(p.DIRECT)

#シミュレーションの状態をリセット
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#重力の設定
p.setGravity(0,0,-10)

#どのくらいの時間間隔でシミュレーションの計算を行うか(ここでは240分の１秒)
#timestep = 1. / 240.
#p.setTimeStep(timestep)

#床の読み込み
planeId = p.loadURDF("plane.urdf")
#momo_envId = p.loadURDF("sampleM.urdf", [0,0,-0.5])

#Racecarの初期状態(位置と向き)を設定
StartPos = [2,0,0] 
StartOrient = p.getQuaternionFromEuler([0,0,3.14]) 

StartPos2 = [0,0,0] 
StartOrient2 = p.getQuaternionFromEuler([0,0,3.14]) 

#Racecarの読み込み
#carId = p.loadURDF("/content/bullet3/data/racecar/racecar.urdf",StartPos, StartOrient)
carId = p.loadURDF("/data/racecar/racecar.urdf", StartPos, StartOrient)
sample_carId = p.loadURDF("/sample/urdf/urdf_sample.urdf", StartPos2, StartOrient2)

#画像の取得
#width, height, rgbImg, depthImg, segImg = p.getCameraImage(360,240)

#画像の表示
#plt.imshow(rgbImg)
#plt.show()

useRealTimeSimulation = 0

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

while 1:
  if (useRealTimeSimulation):
    p.setGravity(0, 0, -10)
    p.setJointMotorControl2(carId, 2, p.VELOCITY_CONTROL, targetVelocity=10)
    p.setJointMotorControl2(carId, 3, p.VELOCITY_CONTROL, targetVelocity=10)
    sleep(0.01)  # Time in seconds.
  else:
    p.stepSimulation()
