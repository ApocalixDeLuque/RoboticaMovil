import time
import math as m
import numpy as np
from scipy.interpolate import splprep, splev
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def convert_velocity_to_wheel_speeds(linear_velocity, angular_velocity, wheel_radius, wheel_distance):
  right_wheel_speed = linear_velocity/wheel_radius + wheel_distance*angular_velocity/(2*wheel_radius)
  left_wheel_speed = linear_velocity/wheel_radius - wheel_distance*angular_velocity/(2*wheel_radius)
  return right_wheel_speed, left_wheel_speed

def calculate_angle_difference(angle1, angle2):
  """
  Compute the angle difference, angle2-angle1, restricting the result to the [-pi,pi] range
  """
  angle_magnitude = m.acos(m.cos(angle1)*m.cos(angle2)+m.sin(angle1)*m.sin(angle2))
  angle_direction = m.cos(angle1)*m.sin(angle2)-m.sin(angle1)*m.cos(angle2)
  return m.copysign(angle_magnitude, angle_direction)

print('Program started')

client = RemoteAPIClient()
simulation = client.getObject('sim')

left_motor=simulation.getObject("/PioneerP3DX/leftMotor")
right_motor=simulation.getObject("/PioneerP3DX/rightMotor")
robot = simulation.getObject("/PioneerP3DX")

simulation.startSimulation()

# Define the control points for the B-spline
ttime = 1.0
xarr = np.array([0.0, 1.0, 2.0, 1.0, 0.0])
yarr = np.array([0.0, 1.0, 0.0, -1.0, 0.0])
tarr = np.linspace(0, ttime, len(xarr))

# Generate the B-spline
tck, u = splprep([xarr, yarr], s=0)
tnew = np.linspace(0, ttime, 100)
out = splev(tnew, tck)

# Use the B-spline points as the goal points
goal_points = list(zip(out[0], out[1]))

velocity_gain = 0.1
heading_gain = 15
wheel_radius = 0.5*0.195
wheel_distance = 0.3110

for goal_x, goal_y in goal_points:
  previous_error = 1000
  while previous_error > 0.1:
    robot_position = simulation.getObjectPosition(robot, -1)
    robot_orientation = simulation.getObjectOrientation(robot, -1)
    previous_error = m.sqrt((goal_x-robot_position[0])**2 + (goal_y-robot_position[1])**2)
    goal_angle = m.atan2(goal_y-robot_position[1], goal_x-robot_position[0])
    heading_error = calculate_angle_difference(robot_orientation[2], goal_angle)
    print('Distance to goal: {}   Heading error: {}'.format(previous_error, heading_error))

    linear_velocity = velocity_gain*previous_error
    angular_velocity = heading_gain*heading_error

    right_wheel_speed, left_wheel_speed = convert_velocity_to_wheel_speeds(linear_velocity, angular_velocity, wheel_radius, wheel_distance)
    simulation.setJointTargetVelocity(left_motor, left_wheel_speed)
    simulation.setJointTargetVelocity(right_motor, right_wheel_speed)

simulation.stopSimulation()