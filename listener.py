import rospy, roslib, tf
from geometry_msgs.msg import TransformStamped as ts
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3Stamped
import libardrone
import time
from threading import Timer
drone = None
goalPos = (0,0, 3)
def createBroadcaster(data):
    br = tf.TransformBroadcaster()
    br.sendTransform(data.transform)

def createListener(data):
    listener = tf.TransformListener()
    v = vout = Vector3Stamped
    v.point = data.transform.translation
    try:
        listener.transformVector3(data.child_frame_id, v,vout)
    except(e):
        pass
    print (vout.x, vout.y, vout.z)

def getNewSpeeds(xyz):
    rightSpeed = 0
    forwardSpeed = 0
    upSpeed = 0
    x = xyz.x
    y = xyz.y
    z = xyz.z
    xDiff = abs(goalPos[0] - x)
    if xDiff > .004:
        if goalPos[0] > x:
            rightSpeed = xDiff/2.0
        else:
            rightSpeed = -xDiff/2.0
    yDiff = abs(goalPos[1] - y)
    if yDiff > .004:
        if goalPos[1] > y:
            forwardSpeed = yDiff/2.0
        else:
            forwardSpeed = -yDiff/2.0
    zDiff = abs(goalPos[2] - z)
    if zDiff > .004:
        if goalPos[2] > z:
            upSpeed = zDiff/2.0
        else:
            upSpeed = -zDiff/2.0
    return (rightSpeed, forwardSpeed, upSpeed)        

def callback(data):
    if time.time() - start_time > 1000000:
        perfLand()    
    else:
        createBroadcaster(data)
        createListener(data)
        #(rightSpeed, forwardSpeed, upSpeed) = getNewSpeeds(data.transform.translation)
        #time.sleep(2)
        #print data.transform.translation
        #print (rightSpeed, forwardSpeed, upSpeed)
        #print
        #def perform_op(move_right, move _forward, move_up, rotate_left_or_right):
        #drone.perform_op(rightSpeed, forwardSpeed, upSpeed, 0)
    #rospy.loginfo(rospy.get_caller_id()+"I heard \n %s",data.transform)

def perfLand():
    drone.halt()
    time.sleep(1)
    drone.land()
    time.sleep(2)
    exit()

def perfTakeoff():
    global drone
    drone = libardrone.ARDrone()
    time.sleep(2)
    drone.takeoff()
    time.sleep(5)

def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    
    #perfTakeoff()
    global start_time
    start_time = time.time()
    rospy.init_node('listener', anonymous=True)
        
    rospy.Subscriber("/vicon/EEQUAD149/EEQUAD149", ts, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
