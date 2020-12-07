#!/usr/bin/env python3
# Written by Nikolas Pardoe, using provided template code for the keypress registration
# 
# 


import sys, select, termios, tty, rospy
from geometry_msgs.msg import Twist, Vector3

#Provided function definition for getting keypress
def getKey(key_timeout):
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

rospy.init_node('pardo020_teleoperation')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

#Init messages. Want to be able to change it
linear = Vector3()
angular = Vector3()
msg = Twist(linear,angular)

#tuning parameters
decayLinear = .005
decayAngular = .25
rampLinear = decayLinear + .025
rampAngular = decayAngular + .25
linearMax = 1
angularMax = 3

#Setup for key listening
settings = termios.tcgetattr(sys.stdin)

while not rospy.is_shutdown():
	#Get key and apply action based on it
	key = getKey(0.1)
	
	# If keypress is not empty, print out the key.
	if key != '':
		print('Pressed '+key)

	# If keypress is Crtl+C, break loop and exit.
	if key == '\x03':
		break
	
	if (key == 'w'):
		msg.linear.x += rampLinear
		if (msg.linear.x < 0): msg.linear.x = 0
	elif (key == 's'):
		msg.linear.x -= rampLinear
		if (msg.linear.x > 0): msg.linear.x = 0
	elif (key == 'a'):
		msg.angular.z += rampAngular
		if (msg.angular.z < 0): msg.angular.z = 0
	elif (key == 'd'):
		msg.angular.z -= rampAngular
		if (msg.angular.z > 0): msg.angular.z = 0
	elif (key == 'x'):
		msg.linear.x = 0
		msg.angular.z = 0
		
	#linear decay
	if (msg.linear.x >= decayLinear):
		msg.linear.x -= decayLinear
	elif (msg.linear.x <= -decayLinear):
		msg.linear.x += decayLinear
	else:
		msg.linear.x = 0
	#angular decay
	if (msg.angular.z >= decayAngular):
		msg.angular.z -= decayAngular
	elif (msg.angular.z <= -decayAngular):
		msg.angular.z += decayAngular
	else:
		msg.angular.z = 0
	
	#bounds
	msg.linear.x = max(-linearMax,min(msg.linear.x,linearMax))
	msg.angular.z = max(-angularMax,min(msg.angular.z,angularMax))
	
	#publish message, sleep
	pub.publish(msg)
	rate.sleep()

	
# close stream after termination
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

