import rospy, roslib
from geometry_msgs.msg import TransformStamped as ts
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3Stamped
import libardrone
import time
from threading import Timer
drone = None
goalPos = (.4,.4, 2.5)
prevTime = None
prevPos = None
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

#velocity is m/s
def calcVelocity(xyz, now):  
    x = xyz.x
    y = xyz.y
    z = xyz.z
    vX,vY,vZ = (0,0,0)
    global prevTime, prevPos  
    if prevTime:
        #normalized by max speed of 5m/s
        vX = (x - prevPos[0])/(now - prevTime)/5.0
        vY = (y - prevPos[1])/(now - prevTime)/5.0
        vZ = (z - prevPos[2])/(now - prevTime)/5.0

    prevTime = now
    prevPos = (x,y,z)

    return (vX, vY, vZ)


def getNewSpeeds(xyz):
    rightSpeed = 0
    forwardSpeed = 0
    upSpeed = 0
    x = xyz.x
    y = xyz.y
    z = xyz.z
    xDiff = abs(goalPos[0] - x)
    if xDiff > .0004:
        if goalPos[0] > x:
            rightSpeed = xDiff
        else:
            rightSpeed = -xDiff
    yDiff = abs(goalPos[1] - y)
    if yDiff > .0004:
        if goalPos[1] > y:
            forwardSpeed = yDiff
        else:
            forwardSpeed = -yDiff
    zDiff = abs(goalPos[2] - z)
    if zDiff > .0004:
        if goalPos[2] > z:
            #battery issues make it so we need to /2 since the up speed is not quite 5 more like 2m/s
            upSpeed = zDiff
        else:
            upSpeed = -zDiff
    return (rightSpeed, forwardSpeed, upSpeed)        

def callback(data):
    now = time.time()
    if time.time() - start_time > 15:
        perfLand()    
    else:
        positionData = data.transform.translation
        #createBroadcaster(data)
        #createListener(data)
        (vX, vY, vZ) = calcVelocity(positionData, now)
        (pX, pY, pZ) = getNewSpeeds(positionData)
        rightSpeed = pX - .9 * vX#(pX + vX)/2
        forwardSpeed = pY - .9 * vY#(pY + vY)/2
        upSpeed = pZ - .9 * vZ#(pZ + vZ)/2
        #time.sleep(2)
        #print data.transform.translation
        print positionData
        print (rightSpeed, forwardSpeed, upSpeed)
        print
        #def perform_op(move_right, move _forward, move_up, rotate_left_or_right):
        drone.perform_op(rightSpeed, forwardSpeed, upSpeed, 0)
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
    
    perfTakeoff()
    global start_time
    start_time = time.time()
    rospy.init_node('listener', anonymous=True)
        
    rospy.Subscriber("/vicon/EEQUAD149/EEQUAD149", ts, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
