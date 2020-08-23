#-*-coding:utf-8-*-
import pybullet as p
import time
import pybullet_data

# GUI 에 연결
physicsClient = p.connect(p.GUI)

# 중력 가속도
p.setGravity(0, 0, -9.8)

# pybullet 내장 model 들을 로드
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setTimeStep(1/200)



cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
wheel_indices = [0,1,2,3]
hinge_indicies = [0,2]


planeID = p.loadURDF("plane.urdf")
carID = p.loadURDF("./simplecar.urdf", cubeStartPos, cubeStartOrientation)
number_of_joints=p.getNumJoints(carID)


angle =p.addUserDebugParameter("Steering",-0.5,0.5,0) # 디버그 창의 좌표
throttle = p.addUserDebugParameter("Throttle",0,20,0)
for i in range(10000):
    # ----- code here

    user_angle      = p.readUserDebugParameter(angle)
    user_throttle   = p.readUserDebugParameter(throttle)

    for joint_index in wheel_indices:
        p.setJointMotorControl2(carID, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    # ----- code end
    p.stepSimulation() # 명령 적용 후 rendering 1번.
    time.sleep(1 / 200)

p.disconnect(physicsClient)