#!/usr/bin/env python3
# Written by Nikolas Pardoe for CSCI 5551
# State machine got messy toward the end to help with corner cases, but the basic
# navigation algorithm is:
# --Calculate the nearest obstacle and direction to it
# --If it's below a certain threshold, face it and back up until further than another threshold
# --If it's closer than a threshold and between the turtlebot and the goal, turn 90 degrees from
#     the obstacle (in whichever direction results in a heading closest to moving toward goal)
# --If none of these other conditions are met, face toward the goal
# --Once within a tolerance of the goal, set linear velocity zero and set heading to goal heading
#     As soon as the heading is within the tolerance call the goal reached service
# This algorithm works very efficientlyas long as all obstacles are convex and 
# sufficiently spaced for the turtlebot to pass between them, which is true for this map
#
# Basic code layout:
# -Obstacle detection/avoidance and my modified bug algorithm are run in the scan callback
#       so that it runs as soon as the latest obstacle data is obtained, and only once per scan
# -Location is updated in global variables when the odometry setpoint arrives
# -Controlling the yaw to its setpoint (simple P control) is done in the main loop at the end.
#       Goal condition checking and calling the goal service is also done here.

import sys, select, termios, tty, rospy, tf, math
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

#Init messages. Want to be able to change it
velocitycmd = Twist(Vector3(),Vector3())
yaw = 0
x = 0
y = 0
#goals
goalset = False
goalx = 0
goaly = .5
goalyaw = 0 #not sure if I need this or not
goaldir = 0;
goaldist = math.inf;
finalpositioning = False
creatingspace = False
#yaw setpoint for control loop
yawset = 0
#tuning parameters/constants

#calculate yaw error, accounting for heading loop
def circleerror(goalyaw, yaw):
    def comperror(error):
        if error > math.pi:
            return comperror(error-2*math.pi)
        elif error < -math.pi:
            return comperror(error + 2*math.pi)
        else:
            return error
    return comperror(goalyaw - yaw)

#subscription callback definitions
def scancallback(scanmsg):
    global yawset, goalset, velocitycmd, finalpositioning,creatingspace
    #parse scan data, use closest obstacle and vehicle position/heading and goal
    #to update the velocity and yaw setpoints
    if not goalset:
        velocitycmd.linear.x = 0
        return

    #find direction to nearest obstacle
    index = scanmsg.ranges.index(min(scanmsg.ranges))
    minrange = scanmsg.ranges[index]
    #obstacle heading in global frame
    obstacleheading = yaw + index*scanmsg.angle_increment
    blockinggoal = abs(circleerror(goaldir, obstacleheading)) < math.pi/2

    #trigger obstacle avoidance if necessary
    if creatingspace or (minrange < .18 and minrange < goaldist):
        rospy.logdebug('creatingspace')
        creatingspace = True
        yawset = obstacleheading
        #wait until facing obstacle, then back up
        if abs(circleerror(obstacleheading, yaw)) < .15:
            velocitycmd.linear.x = -.1
        else:
            velocitycmd.linear.x = 0
        #exit space creation when far enough away or further from obstacle than goal
        if minrange > goaldist or minrange > .35:
            creatingspace = False

    if minrange < .35 and minrange < goaldist and blockinggoal and not creatingspace:
        if abs(circleerror(obstacleheading,goaldir)) < math.pi/20:
            yawset = obstacleheading - math.pi/2
        else:
            yawset = obstacleheading - math.copysign(math.pi/2,circleerror(obstacleheading,goaldir))
    elif not creatingspace:
        yawset = goaldir

    #velocity control code
    if goaldist > .005 and not (finalpositioning or creatingspace):
        vmax = .15
        k = 1
        if circleerror(yawset,yaw) < math.pi/8:
            v = math.cos(4*circleerror(yawset,yaw))*k*goaldist
        else:
            v = 0
        velocitycmd.linear.x = max(0,min(v,vmax))
    elif not creatingspace:
        velocitycmd.linear.x = 0
        yawset = goalyaw
        finalpositioning = True
    #math.copysign(min(maxvel,abs(v)),v) #limit the velocity absolute value



def odomcallback(odommsg):
    global x,y,yaw, goaldir,goaldist
    #basically just update the current position and yaw (do the quaternion conversion here)
    Q = odommsg.pose.pose.orientation
    q = [Q.x,Q.y,Q.z,Q.w]
    #convert to roll, pitch, yaw
    angles = tf.transformations.euler_from_quaternion(q, axes='sxyz')
    # Yaw is -pi < yaw < pi, 0 is "north" (+x, toward turtleworld head)
    yaw = angles[2]
    x = odommsg.pose.pose.position.x
    y = odommsg.pose.pose.position.y
    #calculate direction to goal
    goaldir = math.atan2(goaly-y,goalx-x)
    goaldist = math.sqrt((goaly-y)*(goaly-y)+(goalx-x)*(goalx-x))

def goalcallback(goalmsg):
    global goalx,goaly,goalyaw,goalset, finalpositioning
    #Create the goal setpoints. Reject if navigation is already in progress.
    if goalset: 
        rospy.logdebug('Rejecting new goal, previous goal not yet reached')
    else:
        rospy.logdebug('Setting goal')
        Q = goalmsg.pose.orientation
        q = [Q.x,Q.y,Q.z,Q.w]
        #convert to roll, pitch, yaw
        angles = tf.transformations.euler_from_quaternion(q, axes='sxyz')
        # Yaw is -pi < yaw < pi, 0 is "north" (+x, toward turtleworld head)
        goalyaw = angles[2]
        goalx = goalmsg.pose.position.x
        goaly = goalmsg.pose.position.y
        #calculate direction to goal
        goaldir = math.atan2(goaly-y,goalx-x)
        goaldist = math.sqrt((goaly-y)*(goaly-y)+(goalx-x)*(goalx-x))
        finalpositioning = False
        goalset = True

#initiate node, publishers, subscribers
rospy.init_node('pardo020navigation', log_level=rospy.DEBUG)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/scan', LaserScan, scancallback)
rospy.Subscriber('/odom', Odometry, odomcallback)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalcallback)
goal_reached = rospy.ServiceProxy('/csci_5551/goal_reached',Trigger)
rate = rospy.Rate(10)


#main control loop, outputs both forward and angular velocity based on the setpoints created by the scan
while not rospy.is_shutdown():
    #check if distance to goal below threshold, if so check the yaw threshold if applicable,
    #then call the goal reached service. If yaw needs adjustment set angular rotation,
    #publish, and use sleep and continue to skip to next loop iteration

    #debugging messages
    # rospy.logdebug(goaldist)

    #check if goal reached
    if finalpositioning and circleerror(yawset,yaw) < .001:
         goalset = False
         finalpositioning = False
         status = goal_reached()
         rospy.logdebug('Goal reached!')


    #calculate the angular velocity command
    k = 2
    omegalim = 1.5
    omega = circleerror(yawset,yaw)*k
    omega = math.copysign(min(omegalim,abs(omega)),omega) #limit the yaw command absolute value
    velocitycmd.angular.z = omega
	#publish message, sleep
    pub.publish(velocitycmd)
    rate.sleep() #10Hz
