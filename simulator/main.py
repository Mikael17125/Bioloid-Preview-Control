import pybullet as p
import time
import pybullet_data

# Start pybullet simulation
p.connect(p.GUI)
# p.connect(p.DIRECT) # don't render

# load urdf file path
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# load urdf and set gravity
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
bioloid_init_pos = [0,0,0.3]
bioloid_init_ori = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("assets/bioloid.urdf", bioloid_init_pos, bioloid_init_ori)

# step through the simluation
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()