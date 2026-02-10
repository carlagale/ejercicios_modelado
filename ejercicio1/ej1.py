import pybullet as p
import pybullet_data
import time

GRAVITY = -9.8

# Conectar a PyBullet
p.connect(p.GUI)
p.setGravity(0, 0, GRAVITY)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Cargar robot
robotId = p.loadURDF("ej1.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Obtener IDs de los joints en un diccionario
numJoints = p.getNumJoints(robotId)
jointInfo = {}
for i in range(numJoints):
    info = p.getJointInfo(robotId, i)
    joint_name = info[1].decode("utf-8")
    jointInfo[joint_name] = i


# Crear sliders para los joints
base2vertical_slider = p.addUserDebugParameter("Vertical", -0.8, 0.8, 0)
vertical2horizontal_slider = p.addUserDebugParameter("Horizontal", -3.14, 3.14, 0)

# Bucle principal
while True:
    # Leer valores de los sliders
    base2vertical_val = p.readUserDebugParameter(base2vertical_slider)
    vertical2horizontal_val = p.readUserDebugParameter(vertical2horizontal_slider)

    # Aplicar valores a los joints usando el diccionario
    p.setJointMotorControl2(robotId, jointInfo["base2vertical"], p.POSITION_CONTROL, targetPosition=base2vertical_val)
    p.setJointMotorControl2(robotId, jointInfo["vertical2horizontal"], p.POSITION_CONTROL, targetPosition=vertical2horizontal_val)

    p.stepSimulation()
    time.sleep(1./240.)
