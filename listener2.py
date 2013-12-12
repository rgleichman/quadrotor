import rospy, roslib, math
from geometry_msgs.msg import TransformStamped as ts
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3Stamped
objpos = None
def callback(data):
	global objpos
	objpos = data.transform.translation

def callback2(data):
	#print objpos
	#print
	if (objpos != None):
		quadpos = data.transform.translation
		tmp = [quadpos.x-objpos.x, quadpos.y-objpos.y, quadpos.z-objpos.z]
		distance = math.sqrt(tmp[0]*tmp[0]+tmp[1]*tmp[1]+tmp[2]*tmp[2])
		if (distance < 0.5):
			print "TOO CLOSE WTF"
		else:
			print "yay"
	else:
		print "Stick MIA"


def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("/vicon/wand/wand", ts, callback)
    rospy.Subscriber("/vicon/EEQUAD149/EEQUAD149", ts, callback2)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
