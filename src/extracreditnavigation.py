#!/usr/bin/env python3
# Written by Nikolas Pardoe for CSCI 5551
# The setup for the extra credit currently relies heavily on
# standard libraries and the tutorial http://wiki.ros.org/navigation/Tutorials/RobotSetup
# I am working to run the navigation indpendently of move_base but ran out of time.

import sys, select, termios, tty, rospy, tf, math
from geometry_msgs.msg import Twist, Vector3, PoseStamped
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
donenavigating = True
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
        donenavigating = False

def cmdvelcallback(cmdmsg):
    global donenavigating
    donenavigating = False

#initiate node, publishers, subscribers
rospy.init_node('pardo020navigation', log_level=rospy.DEBUG)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/cmd_vel', Twist, cmdvelcallback)
rospy.Subscriber('/odom', Odometry, odomcallback)
rospy.Subscriber('/move_base_simple/goal', PoseStamped, goalcallback)
goal_reached = rospy.ServiceProxy('/csci_5551/goal_reached',Trigger)
rate = rospy.Rate(1)

#goal completion checking
while not rospy.is_shutdown():

    #check if goal reached based on navigator no longer sending commands
    if goalset and donenavigating:
         goalset = False
         status = goal_reached()
         rospy.logdebug('Goal reached!')
    else:
        donenavigating = True

    rate.sleep() #10Hz
