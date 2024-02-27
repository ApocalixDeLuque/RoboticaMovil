import math as m
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def convert_velocity_to_wheel_speeds(linear_velocity, angular_velocity, wheel_radius, wheel_distance):
    right_wheel_speed = (2 * linear_velocity + angular_velocity * wheel_distance) / (2 * wheel_radius)
    left_wheel_speed = (2 * linear_velocity - angular_velocity * wheel_distance) / (2 * wheel_radius)
    return right_wheel_speed, left_wheel_speed

def calculate_angle_difference(angle1, angle2):
    return m.atan2(m.sin(angle1 - angle2), m.cos(angle1 - angle2))


client = RemoteAPIClient()
simulation = client.getObject('sim')

left_motor=simulation.getObject("/PioneerP3DX/leftMotor")
right_motor=simulation.getObject("/PioneerP3DX/rightMotor")
robot = simulation.getObject("/PioneerP3DX")

velocity_gain = 0.1
heading_gain = 15
wheel_radius = 0.5*0.195
wheel_distance = 0.3110

simulation.startSimulation()

# Definir la posición objetivo en {C}
x_c_goal, y_c_goal = 2.5, 1.5

# Transformar de {C} a {B}
x_b_goal = x_c_goal * m.cos(m.pi/4) - y_c_goal * m.sin(m.pi/4)
y_b_goal = x_c_goal * m.sin(m.pi/4) + y_c_goal * m.cos(m.pi/4)

# Transformar de {B} a {W}
robot_position = simulation.getObjectPosition(robot, -1)
robot_orientation = simulation.getObjectOrientation(robot, -1)
x_w_goal = robot_position[0] + x_b_goal * m.cos(robot_orientation[2]) - y_b_goal * m.sin(robot_orientation[2])
y_w_goal = robot_position[1] + x_b_goal * m.sin(robot_orientation[2]) + y_b_goal * m.cos(robot_orientation[2])

# Definir la ganancia del controlador proporcional
kp = 0.5

while True:
    # Obtener la posición actual del robot
    robot_position = simulation.getObjectPosition(robot, -1)
    x_w, y_w = robot_position[0], robot_position[1]

    # Calcular el error de posición
    error_x = x_w_goal - x_w
    error_y = y_w_goal - y_w

    # Calcular la distancia al objetivo
    distance = m.sqrt(error_x**2 + error_y**2)

    # Si la distancia al objetivo es menor que un umbral, detener el robot
    if distance < 0.1:
        simulation.setJointTargetVelocity(left_motor, 0)
        simulation.setJointTargetVelocity(right_motor, 0)
        break

    # Calcular la velocidad linear y angular deseadas
    linear_velocity = kp * distance
    angular_velocity = m.atan2(error_y, error_x)

    # Convertir las velocidades deseadas en velocidades de rueda
    right_wheel_speed, left_wheel_speed = convert_velocity_to_wheel_speeds(linear_velocity, angular_velocity, wheel_radius, wheel_distance)

    # Establecer las velocidades de las ruedas
    simulation.setJointTargetVelocity(left_motor, left_wheel_speed)
    simulation.setJointTargetVelocity(right_motor, right_wheel_speed)

    # Esperar un poco antes de la próxima iteración
    time.sleep(0.01)

simulation.stopSimulation()