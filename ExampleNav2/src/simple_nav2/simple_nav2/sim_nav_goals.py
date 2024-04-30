#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def main():
    rclpy.init()

    nav = BasicNavigator()
    nav.declare_parameter('map', rclpy.Parameter.Type.STRING) 

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.0
    initial_pose.pose.position.y = 1.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    nav.setInitialPose(initial_pose)
    
    # Set up the publisher
    nav.init_pose_pub = nav.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    pwcs = PoseWithCovarianceStamped()
    pwcs.header = initial_pose.header
    pwcs.pose.pose = initial_pose.pose
    pwcs.pose.covariance=[float(0.2), float(0), float(0), float(0), float(0), float(0), 
                          float(0), float(0.2), float(0), float(0), float(0), float(0), 
                          float(0), float(0), float(0), float(0), float(0), float(0), 
                          float(0), float(0), float(0), float(0), float(0), float(0), 
                          float(0), float(0), float(0), float(0), float(0), float(0), 
                          float(0), float(0), float(0), float(0), float(0), float(0.06853891909122467)]
    nav.init_pose_pub.publish(pwcs)

    # Wait for navigation to fully activate, since autostarting nav2
    nav.waitUntilNav2Active(localizer='bt_navigator')
#    nav.waitUntilNav2Active(localizer='amcl')
    nav.init_pose_pub.publish(pwcs)

    rate = nav.create_rate(4, nav.get_clock())
    nav.counter = 0
    while rclpy.ok():            
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = nav.get_clock().now().to_msg()

        if nav.counter % 2 == 0:
            # First pose
            theta = 0.0  # Orientation angle
            msg.pose.position.x = -0.5
            msg.pose.position.y = 0.5
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(theta/2)
            msg.pose.orientation.w = math.cos(theta/2)
        else:
            # Second pose
            theta = math.pi  # Orientation angle
            msg.pose.position.x = -3.0
            msg.pose.position.y = 1.0
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(theta/2)
            msg.pose.orientation.w = math.cos(theta/2)
                        
        nav.get_logger().info('Goal selected: ' + str(msg.pose.position.x) + ' ' +
                  str(msg.pose.position.y))
        nav.goToPose(msg)

        nav.counter += 1

        first = True
        while not nav.isTaskComplete():
            # Do something with the feedback
            feedback = nav.getFeedback()
            if feedback and first:
                nav.get_logger().info('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')
                first = False
                
        # Do something depending on the return code
        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            result_str = 'succeeded'
        elif result == TaskResult.CANCELED:
            result_str = 'cancelled'
        elif result == TaskResult.FAILED:
            result_str = 'failed'
        else:
            result_str = 'invalid'

        nav.get_logger().info('Goal ' + result_str + ': ' + str(msg.pose.position.x) + ' ' +
                  str(msg.pose.position.y))
        
        rclpy.spin_once(nav)
        rate.sleep()

    # Exit
    nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
