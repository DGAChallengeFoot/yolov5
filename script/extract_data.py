#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState
import rospy
import os.path
from datetime import datetime
import time


class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name

class State:

    _blockListDict = {
        'block_a': Block('drone1', 'base_link'),
        'block_b': Block('drone2', 'base_link'),
        'block_c': Block('drone3', 'base_link'),
        'block_d': Block('drone4', 'base_link'),
    }

    def storeDataInCsv(self, file_path):
        file1 = open(file_path, "w")
        count = 0
        while not rospy.is_shutdown():
            toFile = datetime.now().isoformat()
            try:
                model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                for block in self._blockListDict.itervalues():
                    blockName = str(block._name)
                    resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
                    toFile = toFile + ',' + blockName
                    toFile = toFile + ',' + str(resp_coordinates.pose.position.x)
                    toFile = toFile + ',' + str(resp_coordinates.pose.position.y)
                    toFile = toFile + ',' + str(resp_coordinates.pose.position.z)
            except rospy.ServiceException as e:
                rospy.logerr("Get Model State service call failed: {0}".format(e))
                raise e
            file1.write(toFile + '\n')
            time.sleep(0.5)
            count += 1
            if count == 10 :
                break

        file1.close()


	


if __name__ == '__main__':
    save_path = "/home/ubuntu/Documents/"
    mydate = datetime.now().isoformat()
    file_name = "DataSim" + mydate[2:19] + ".csv"
    complete_name = save_path + file_name
    print("Beginnig data storage in file: " + complete_name ) 
    storage = State()
    storage.storeDataInCsv(complete_name)

