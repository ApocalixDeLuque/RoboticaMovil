"""
Usando CoppeliaSim se debe programar un robot pioneer p3dx para que realice un cuadrado de 3 metros de lado.
Se debe mover el robot 3 metros hacia delante, rotar 90 grados a la izquierda y repetir el proceso 4 veces para formar un cuadrado.
"""

import time
import math as m

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

client = RemoteAPIClient()
sim = client.getObject('sim')

print('Program started')

motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot  = sim.getObject("/PioneerP3DX")

sim.startSimulation()
r = 0.5*0.195
L = 2.0*0.1655

position = sim.getObjectPosition(robot)
print(position)

for i in range(4):
    tstart = sim.getSimulationTime()
    while sim.getSimulationTime() - tstart < 12:
        ur, ul = v2u(0.25, 0, r, L)
        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)

    tstart = sim.getSimulationTime()
    while sim.getSimulationTime() - tstart < 8:
        ur, ul = v2u(0, m.pi/16.0, r, L)
        sim.setJointTargetVelocity(motorL, ul)
        sim.setJointTargetVelocity(motorR, ur)

# sim.pauseSimulation()
# time.sleep(1)
        

position = sim.getObjectPosition(robot)
print(f'Posicion final robot: {position}')


sim.stopSimulation()