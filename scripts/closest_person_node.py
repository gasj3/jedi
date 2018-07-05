#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from jedi.msg import PersonDistance




def distance (x1, y1, x2, y2):
	xd = x1 -x2
	yd = y1 - y2
	return math.sqrt(xd*xd + yd*yd)

class PersonMonitor(object):
	def __init__(self, pub, people):
		self._pub = pub
		self._people = people


	def callback(self, msg): 
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y
		closest_name = None
		closest_distance = None
		for l_name, l_x, l_y in self._people:
			dist = distance(x,y, l_x, l_y)
			if closest_distance is None or dist < closest_distance:
				closest_name	= l_name
				closest_distance = dist
		ld = PersonDistance()
		ld.name = closest_name
		ld.distance = closest_distance
		self._pub.publish(ld)		
		#rospy.loginfo('closest: {}'.format(closest_name))
	#do something with closest

#rospy.loginfo('x: {}, y: {}'.format(x,y))
def main():

	rospy.init_node('closest_person_node')
	people = []
	
	people.append(("Person1", -1.36, 6.3)); #Pessoa da sala inferior direita
	people.append(("Person2", 2.58, -1.07)); #Pessoa da sala superior direita
	people.append(("Person3", 2.31, 8.80)); #Pessoa na sala da esquerda
	people.append(("Person4", 3.12, 5.56)); #Pessoa da sala do meio superior
	pub = rospy.Publisher('closest_person', PersonDistance, queue_size=10)
	monitor = PersonMonitor (pub, people)
	
	rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, monitor.callback)
	rospy.spin()

if __name__ == '__main__':
	main()
