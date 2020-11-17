import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math


kp = 1
ki = 0.1
Int = 0
kd = 0.5
old_error = 0
estado = 1

#soma dos numeros da matricula:
#Erica = 29
#Joao = 25
#Orlando = 21
#Patricia = 38
#Media = 28.25
#freq = Media
#tempo = 1/freq
tempo = 1/28.25

odom = Odometry()
scan = LaserScan()

rospy.init_node('cmd_node')

# Auxiliar functions ------------------------------------------------
def getAngle(msg):
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    yaw = euler[2]*180.0/math.pi
    return yaw

# CALLBACKS ---------------------------------------------------------
def odomCallBack(msg):
    global odom
    odom = msg
    
def scanCallBack(msg):
    global scan
    scan = msg
#--------------------------------------------------------------------

# TIMER - Control Loop ----------------------------------------------
def timerCallBack(event):
    global kp, ki, kd
    global Int
    global old_error
    global estado
    """
    yaw = getAngle(odom)
    setpoint = -45
    error = (setpoint - yaw)
    
    if abs(error) > 180:
        if setpoint < 0:
            error += 360 
        else:
            error -= 360
    """
    """
    setpoint = (-1,-1)
    position = odom.pose.pose.position
    dist = setpoint[0] - position.x #math.sqrt((setpoint[0] - position.x)**2 + (setpoint[1] - position.y) **2)
    error = dist
    """
    setpoint = 0.5
    scan_len = len(scan.ranges)
    
    if estado == 1:
        if scan_len > 0:
            yaw = getAngle(odom)
            
            ind = scan.ranges.index(min(scan.ranges))
            inc = 2*math.pi / scan_len
            ang = (ind * inc * 180.0/math.pi) + yaw
            if ang > 180:
                ang -= 360
                
            error = (ang - yaw)
            
            if abs(error) > 180:
                if setpoint < 0:
                    error += 360 
                else:
                    error -= 360
                    
            print(ang, yaw, error)
            
            delta_e = error - old_error
            old_error = error
            
            P = kp*error
            Int += error*tempo
            I = Int * ki
            D = (delta_e/tempo)* kd
            
            control = P+I+D
            if control > 0.5:
                control = 0.5
            elif control < -0.5:
                control = -0.5
        else:
            control = 0
            error = 180
        
        if abs(error) < 0.1:
            Int = 0
            old_error = 0
            control = 0
            estado = 2
        
        msg = Twist()
        msg.linear.x = 0
        msg.angular.z = control
        pub.publish(msg)
        
    
    elif estado == 2:
        if scan_len > 0:
            read = min(scan.ranges)
            print(read)
            error = -(setpoint - read)
            print(error)
            
            delta_e = error - old_error
            old_error = error
            
            P = kp*error
            Int += error*tempo
            I = Int * ki
            D = (delta_e/tempo) * kd
            
            control = P+I+D
            if control > 0.5:
                control = 0.5
            elif control < -0.5:
                control = -0.5
        else:
            control = 0
            error = 1
            
        if abs(error) < 0.1:
            Int = 0
            estado = 3
            old_error = 0
            control = 0
            
        msg = Twist()
        msg.linear.x = control
        pub.publish(msg)
        
    elif estado == 3:
        if scan_len > 0:
            read = min(scan.ranges)
            error = -(setpoint - read)
            if error > 0.1:
                estado = 2
            
            yaw = getAngle(odom)
            ind = scan.ranges.index(min(scan.ranges))
            inc = 2*math.pi / scan_len
            ang = (ind * inc * 180.0/math.pi) + yaw
            if ang > 180:
                ang -= 360
            error = (ang - yaw)
            if error > 0.1:
                estado = 1
            
        msg = Twist()
        msg.linear.x = 0
        msg.linear.z = 0
        pub.publish(msg)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom_sub = rospy.Subscriber('/odom', Odometry, odomCallBack)
scan_sub = rospy.Subscriber('/scan', LaserScan, scanCallBack)

timer = rospy.Timer(rospy.Duration(tempo), timerCallBack)

rospy.spin()