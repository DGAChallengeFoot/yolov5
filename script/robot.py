#!/usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
import numpy as np

class Robot:
    def __init__(self, type, number):
        assert str(type) in {"robot", "drone"}
        self._type = type
        self._name = 'drone{}'.format(number)
        self._isAlive = True
        self._relative_entity_name = 'base_link' ## à clarifier
    
    def get_position(self):
        try:
            model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp_coordinates = model_coordinates(self._name, self._relative_entity_name)
            print('State success = ', resp_coordinates.success)
            return resp_coordinates.pose.position

        except rospy.ServiceException as e:
            rospy.logerr("Get Model State service call failed: {0}".format(e))
            raise e



## need to implement:
# - get position: array de dim 3
# - get actions: array de dim 6
# - move
# - get image

if __name__ == '__main__':
    testRobot = Robot('drone', 1)
    testRobot.get_position()