import rospy, roslib, math, libardrone, time
from geometry_msgs.msg import TransformStamped as ts
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3Stamped
objpos = None
prevTime = None
prevPos = None
start = None

def callback(data):
	global objpos
	objpos = data.transform.translation
	#print "X = " + str(objpos.x)
	#print "Y = " + str(objpos.y)

def callback2(data):
	#print objpos
	#print
	now = time.time()
	global start
	if now - start > 30
		perfLand()
		return
	if (objpos != None):
		quadpos = data.transform.translation
		tmp = [quadpos.x-objpos.x, quadpos.y-objpos.y, quadpos.z-objpos.z]
		distance = math.sqrt(tmp[0]*tmp[0]+tmp[1]*tmp[1]+tmp[2]*tmp[2])
		if (distance < 0.7):
			print "TOO CLOSE WTF"
			for i in tmp:
				i /= distance
			print "X = " + str(tmp[0])
			print "Y = " + str(tmp[1])
			print "Z = " + str(tmp[2])

			drone.perform_op(tmp[0], tmp[1], tmp[2], 0)
		else:
			print "yay"
			#drone.perform_op(0, 0, 0, 0)
			drone.hover()
	else:
		print "Stick MIA"


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
    global start
    start = time.time()

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/vicon/wand/wand", ts, callback)
    rospy.Subscriber("/vicon/EEQUAD1492/EEQUAD1492", ts, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
