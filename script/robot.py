#!/usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors

class Robot:
    def __init__(self, type, number):
        assert str(type) in {"robot", "drone"}
        self._type = type
        self._name = 'drone{}'.format(number)
        self._isAlive = True
        self._relative_entity_name = 'base_link'

    def get_position(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(self._name, self._relative_entity_name)
            result = np.array([resp_coordinates.pose.position.x, resp_coordinates.pose.position.y, resp_coordinates.pose.position.z])
            return result

        except rospy.ServiceException as e:
            rospy.logerr("Get Model State service call failed: {0}".format(e))
            raise e

    def move(self, move_info):
        """ Move robot according to the information given by the array move_info.
        Template: move_info = np.array([distance.x, distance.y, distance.z, speed.x, speed.y, speed.z]) in m and m/s.
        """
        rospy.init_node('move_{}'.format(self._name))

        if self.motor_on():
            velocity_publisher = rospy.Publisher('{}/cmd_vel'.format(self._name), Twist, queue_size=1)
            vel_msg = Twist()
            vel_msg.linear.x = move_info[3]
            vel_msg.linear.y = move_info[4]
            vel_msg.linear.z = move_info[5]
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            while not rospy.is_shutdown():
                t0 = rospy.Time.now().to_sec() * 1e-3
                d_x = 0
                d_y = 0
                d_z = 0
                count = 0
                current_distance = np.sqrt(d_x**2 + d_y**2 + d_z**2)
                distance = np.sqrt(move_info[0]**2 + move_info[1]**2 + move_info[2]**2)
                print("distance : " + str(distance))
                while (current_distance < distance) and not rospy.is_shutdown():
                    if (d_x >= move_info[0]):
                        vel_msg.linear.x = 0.
                    if (d_y >= move_info[1]):
                        vel_msg.linear.y = 0.
                    if (d_z >= move_info[2]):
                        vel_msg.linear.z = 0.

                    t1 = rospy.Time.now().to_sec() * 1e-3
                    d_x = move_info[3] * (t1 - t0)
                    d_y = move_info[4] * (t1 - t0)
                    d_z = move_info[5] * (t1 - t0)
                    current_distance = np.sqrt(d_x**2 + d_y**2 + d_z**2)
                    if count % 1000 == 0: print(t1 - t0)
                    velocity_publisher.publish(vel_msg)
                    
                vel_msg.linear.x = 0.
                vel_msg.linear.y = 0.
                vel_msg.linear.z = 0.
                velocity_publisher.publish(vel_msg)
                break
        else:
            return 0

    def motor_on(self):
        rospy.wait_for_service('{}/enable_motors'.format(self._name))
        try:
            motor_on = rospy.ServiceProxy('{}/enable_motors'.format(self._name), EnableMotors, True)
            turn_on = motor_on(True)
            return turn_on
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

## need to implement:
# - get actions: array de dim 6
# - move
# - get image

if __name__ == '__main__':
    testRobot = Robot('drone', 3)
    move_inf = np.array([2, 2, 2.0, 0.25, 0.25, 0.25])
    testRobot.move(move_inf)
