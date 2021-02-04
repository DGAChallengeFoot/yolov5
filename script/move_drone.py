#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors

# Ros function to move a drone arg =(drone_ID, rotation, number_of_drone, vector_distance, vector_speed=[2, 2, 2])
def move(drone_ID, rotation, number_of_drone, vector_distance, vector_speed=[2, 2, 2]):
    # Starts a new node

    rospy.init_node('move_drone{}'.format(drone_ID))  # cree un node
    # check if motors are on
    if motor_on():
        velocity_publisher = rospy.Publisher('drone{}/cmd_vel'.format(drone_ID), Twist, queue_size=1)

        vel_msg = Twist()

        # Receiveing the user's input
        speed = vector_speed
        distance = vector_distance

        vel_msg.linear.x, vel_msg.linear.ymvel_msg.linear.z = speed[0], speed[1], speed[2]
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = rotation

        while not rospy.is_shutdown():

            # Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0

            # Loop to move the turtle in an specified distance
            while (current_distance < distance) and not rospy.is_shutdown():
                # Publish the velocity
                velocity_publisher.publish(vel_msg)
                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # Calculates distancePoseStamped
                current_distance = speed * (t1 - t0)
            # After the loop, stops the robot
            vel_msg.linear.z = 0.1
            # Force the robot to stop
            velocity_publisher.publish(vel_msg)
        else:
            return 0


# Rosservice function to turn on the drone motors
def motor_on():
    rospy.wait_for_service('drone{}/enable_motors'.format(drone_N))
    try:
        motor_on = rospy.ServiceProxy('drone{}/enable_motors'.format(drone_N), EnableMotors, True)
        turn_on = motor_on(True)
        return turn_on
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    # Receiveing the user's input
    print("Let's move your drone")
    drone_N = input("Select a drone to move. (Options 1, 2, 3, 4): ")
    # Testing our function
    move(drone_N)
