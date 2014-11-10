#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist

class randomMoveObstacleDetector:

  def __init__(self):
#initialize node
    rospy.init_node('qbo_random_move_avoid_obstacles')

    self.wallDetectionDistance = 0.4
    self.leftDistance = 0.
    self.rightDistance = 0.
    self.floorDistance = 0.29
    self.floorInterval = 0.02
    self.floorSeen = True

#create subscribers to srf and floor sensors
    rospy.Subscriber('/distance_sensors_state/front_left_srf10', PointCloud, self.leftSensorCallback)
    rospy.Subscriber('/distance_sensors_state/front_right_srf10', PointCloud, self.rightSensorCallback)
    rospy.Subscriber('/distance_sensors_state/floor_sensor', PointCloud, self.floorSensorCallback)

    rospy.Subscriber('/qbo_random_move/obstacle_avoidance_twist', Twist, self.twistCallback)
    self.twistPublisher = rospy.Publisher("/output_twist", Twist, queue_size=1)


  def leftSensorCallback(self, data):
    #print "left"
#store left sensor distance
    self.leftDistance = data.points[0].x
#ignore if obstacle is too far away
    if self.leftDistance > self.wallDetectionDistance:
      self.leftDistance = 0

  def rightSensorCallback(self, data):
    #print "right"
#store right sensor distance
    self.rightDistance = data.points[0].x
#ignore if obstacle is too far away
    if self.rightDistance > self.wallDetectionDistance:
      self.rightDistance = 0

  def floorSensorCallback(self, data):
    #print "floor ",data.points[0].x
    self.floorSeen = (data.points[0].x < self.floorDistance + self.floorInterval) and (data.points[0].x > self.floorDistance - self.floorInterval)

  def twistCallback(self, data):
    if not self.floorSeen :
      print "no floor ! going back !"
      data.linear.x = -0.1
      data.angular.z = -0.1
    else:
      if self.rightDistance > 0 and self.leftDistance == 0: #right obstacle
        print "right obstacle"
        data.linear.x = min(0.15, data.linear.x) #not too quick
        data.angular.z = max(0.1, data.angular.z)  #turn left
      elif self.rightDistance == 0 and self.leftDistance > 0: #left obstacle
        print "left obstacle"
        data.linear.x = min(0.15, data.linear.x)
        data.angular.z = min(-0.1, data.angular.z)
      elif self.rightDistance > 0 and self.leftDistance > 0: #front obstacle
        print "front obstacle"
        data.linear.x = min(0.15, data.linear.x)
        data.angular.z = min(-0.1, data.angular.z)
    self.twistPublisher.publish(data)

 
    



if __name__ == '__main__':
  try:
    node = randomMoveObstacleDetector()
    rospy.spin()
  except rospy.ROSInterruptException:
    print "interrupted  !"
