import time
import math as m

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def v2u(v, omega, r, L):
    ur = v/r + L*omega/(2*r)
    ul = v/r - L*omega/(2*r)
    return ur, ul

# Establecer conexión con CoppeliaSim
client = RemoteAPIClient()
sim = client.getObject('sim')

print('Program started')

# Obtener manejadores de los motores y el robot
motorL = sim.getObject("/PioneerP3DX/leftMotor")
motorR = sim.getObject("/PioneerP3DX/rightMotor")
robot  = sim.getObject("/PioneerP3DX")

# Iniciar simulación
sim.startSimulation()

# Definir parámetros del robot
r = 0.0975  # Radio de las ruedas (m)
L = 0.331   # Distancia entre ruedas (m)

try:
    for _ in range(4):  # Repetir el patrón de movimiento
        # Avanzar
        tstart = sim.getSimulationTime()
        while sim.getSimulationTime() - tstart < 8:  # Tiempo para avanzar
            ur, ul = v2u(0.25, 0, r, L)
            sim.setJointTargetVelocity(motorL, ul)
            sim.setJointTargetVelocity(motorR, ur)
            time.sleep(0.1)
        
        # Girar con mayor omega
        tstart = sim.getSimulationTime()
        while sim.getSimulationTime() - tstart < 4:  # Ajustar este tiempo según sea necesario
            # Aumentar omega para un giro más pronunciado
            ur, ul = v2u(0, m.pi/2, r, L)  # Intentar con pi/2 para un giro de 90 grados
            sim.setJointTargetVelocity(motorL, ul)
            sim.setJointTargetVelocity(motorR, ur)
            time.sleep(0.1)
finally:
    sim.stopSimulation()

print('Program ended')
