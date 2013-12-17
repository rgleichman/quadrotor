import rospy, roslib
from geometry_msgs.msg import TransformStamped as ts
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3Stamped
import libardrone
import time
import random
from threading import Timer
import math
drone = None
prevTime = None
prevPos = None
goalPositions = [(0,0,1.5)]
goalPos = goalPositions[0]
goalPositions.append(goalPos)
endTime = len(goalPositions) * 20
maneuverStart = None
objpos = None
goalIdx = 0
start_time = None

def genGoalPositions():
    global goalPositions, endTime
    for i in range(5):
        x = random.randint(0,10)/10.0
        y = random.randint(0,10)/10.0
        z = random.randint(5,12)/5.0
        goalPositions.append((x,y,z))
    endTime = len(goalPositions) * 15
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
    if xDiff > .1:
        if goalPos[0] > x:
            rightSpeed = xDiff
        else:
            rightSpeed = -xDiff
    yDiff = abs(goalPos[1] - y)
    if yDiff > .1:
        if goalPos[1] > y:
            forwardSpeed = yDiff
        else:
            forwardSpeed = -yDiff
    zDiff = abs(goalPos[2] - z)
    if zDiff > .0004:
        if goalPos[2] > z:
            upSpeed = zDiff
        else:
            upSpeed = -zDiff
    return (.5 * rightSpeed, .5 * forwardSpeed, upSpeed)        

def callback(data):
    now = time.time()
    global maneuverStart, goalPos, goalIdx, goalPositions, endTime, start_time
    #if not maneuverStart:
    #    maneuverStart = now
    #    start_time = now
    #elif now - maneuverStart > 3:
     #   goalIdx = (goalIdx + 1) % len(goalPositions)
      #  goalPos = goalPositions[goalIdx]
       # maneuverStart = now
    if now - start_time > endTime:
        perfLand()    
        return
    else:
        if (objpos != None):
            quadpos = data.transform.translation
            tmp = [quadpos.x-objpos.x, quadpos.y-objpos.y, quadpos.z-objpos.z]
            distance = math.sqrt(tmp[0]*tmp[0]+tmp[1]*tmp[1]+tmp[2]*tmp[2])
            if (distance < 1):
                (vX, vY, vZ) = calcVelocity(quadpos, now)
                print "TOO CLOSE WTF"
                for i in tmp:
                    i /= distance
                print "X = " + str(tmp[0])
                print "Y = " + str(tmp[1])
                print "Z = " + str(tmp[2])
                rightSpeed = 1.2 * tmp[0] -  vX
                forwardSpeed = 1.2 * tmp[1] - vY
                upSpeed = 1.2 * tmp[2] - vZ
                drone.perform_op(rightSpeed, forwardSpeed, upSpeed, 0)
            else:    
                positionData = data.transform.translation
                (vX, vY, vZ) = calcVelocity(positionData, now)
                (pX, pY, pZ) = getNewSpeeds(positionData)
                rightSpeed = pX - .8 * vX
                forwardSpeed = pY - .8 * vY
                upSpeed = pZ - .8 * vZ
                #time.sleep(2)
                #print positionData
                #print (rightSpeed, forwardSpeed, upSpeed)
                #print
            #def perform_op(move_right, move _forward, move_up, rotate_left_or_right):
            drone.perform_op(rightSpeed, forwardSpeed, upSpeed, 0)
        #rospy.loginfo(rospy.get_caller_id()+"I heard \n %s",data.transform)
        else:
            positionData = data.transform.translation
            (vX, vY, vZ) = calcVelocity(positionData, now)
            (pX, pY, pZ) = getNewSpeeds(positionData)
            rightSpeed = pX - .8 * vX
            forwardSpeed = pY - .8 * vY
            upSpeed = pZ - .8 * vZ
            #time.sleep(2)
            #print positionData
            #print (rightSpeed, forwardSpeed, upSpeed)
            #print
        #def perform_op(move_right, move _forward, move_up, rotate_left_or_right):
            drone.perform_op(rightSpeed, forwardSpeed, upSpeed, 0)
def callbackwand(data):
    global objpos
    objpos = data.transform.translation

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
    #genGoalPositions()
    perfTakeoff()
    global start_time
    start_time = time.time()
    rospy.init_node('listener', anonymous=True)
        
    rospy.Subscriber("/vicon/EEQUAD1492/EEQUAD1492", ts, callback)

    rospy.Subscriber("/vicon/wand/wand", ts, callbackwand)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
