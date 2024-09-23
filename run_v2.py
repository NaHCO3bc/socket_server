#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Char
import math
# from pynput import keyboard
import tf.transformations

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.signal_sub = rospy.Subscriber('/signal', String, self.signal_callback)

        rospy.loginfo("init")
        self.odom_data = None
        self.rate = rospy.Rate(200)  # 10 Hz
        self.green_light = False
        self.run_loop = False
        self.has_rotated = False
        self.has_moved_forward = False
        self.has_moved_backward = False

        self.yaw_offset = 0.0
        self.last_yaw = None

        # Initialize keyboard listener
        # self.listener = keyboard.Listener(on_press=self.on_press)
        # self.listener.start()
        # rospy.loginfo("Keyboard listener started. Press 'q' to exit.")

    def odom_callback(self, data):
        self.odom_data = data
        current_yaw = self.get_yaw_from_orientation(self.odom_data.pose.pose.orientation)
        if self.last_yaw is not None:
            delta_yaw = self.normalize_angle(current_yaw - self.last_yaw)
            if abs(delta_yaw) > math.pi:
                if delta_yaw > 0:
                    self.yaw_offset -= 2 * math.pi
                else:
                    self.yaw_offset += 2 * math.pi
        self.last_yaw = current_yaw


    def signal_callback(self, msg):
        if msg.data == 'GREEN':
            rospy.loginfo("Received GREEN signal")
            self.green_light = True
        elif msg.data == 'RED':
            rospy.loginfo("Received RED signal")
            self.green_light = False

    # def on_press(self, key):
    #     try:
    #         char = key.char
    #         rospy.loginfo("Key pressed: {0}".format(char))
    #         if char == 'w':
    #             self.move_forward(0.5)
    #         elif char == 'b':
    #             self.move_backward(0.5)
    #         elif char == 's':
    #             self.run_loop = True
    #             rospy.loginfo("Started main loop")
    #         elif char == 'q':
    #             rospy.signal_shutdown("User requested shutdown")
    #         elif char == 'r':
    #             self.rotate_180()
    #     except AttributeError:
    #         pass  # Special keys (e.g., shift) are ignored

    def move_forward(self, distance):
        rospy.loginfo("Moving forward by {0} meters".format(distance))
        initial_position = None
        initial_yaw = None
        while initial_position is None:
            if self.odom_data:
                initial_position = self.odom_data.pose.pose.position
                initial_yaw = self.get_continuous_yaw(self.odom_data.pose.pose.orientation)
            self.rate.sleep()

        target_position = initial_position.x + distance
        move_cmd = Twist()

        while True:
            current_position = self.odom_data.pose.pose.position.x
            current_yaw = self.get_continuous_yaw(self.odom_data.pose.pose.orientation)

        # 检查是否达到目标位置
            if (distance > 0 and current_position >= target_position) or (distance < 0 and current_position <= target_position):
                rospy.loginfo("Finish Move")
                break

        # 纠正路径
            if abs(current_yaw - initial_yaw) > 0.3:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.5 * abs(initial_yaw - current_yaw) / (initial_yaw - current_yaw)
                rospy.loginfo("Correcting yaw: current_yaw={0}, initial_yaw={1}".format(current_yaw, initial_yaw))
            else:
                base_vel = 0.5 if distance > 0 else -0.5
                err = abs(current_position - target_position)
                move_cmd.linear.x = base_vel * (err) + base_vel*0.3
                move_cmd.angular.z = 0.5* (current_yaw - initial_yaw)
                rospy.loginfo("Moving: current_position={0}, target_position={1}".format(current_position, target_position))

            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.loginfo("Finished moving forward")
 
    def move_backward(self, distance):
        rospy.loginfo("Moving backward by {0} meters".format(distance))
        self.move_forward(-distance)

    def rotate_180(self):
        rospy.loginfo("Starting 180 degree rotation")
        initial_orientation = None
        while initial_orientation is None:
            if self.odom_data:
                initial_orientation = self.odom_data.pose.pose.orientation
            self.rate.sleep()

        initial_yaw = self.get_continuous_yaw(initial_orientation)
        target_yaw = self.normalize_angle(initial_yaw + math.pi)

        rotate_cmd = Twist()

        while abs(self.normalize_angle(self.get_continuous_yaw(self.odom_data.pose.pose.orientation) - target_yaw)) > 0.1:
            rotate_cmd.angular.z = -1*(self.get_continuous_yaw(self.odom_data.pose.pose.orientation) - target_yaw)
            rospy.loginfo(self.get_continuous_yaw(self.odom_data.pose.pose.orientation) - target_yaw)
            self.cmd_vel_pub.publish(rotate_cmd)
            self.rate.sleep()

        rotate_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(rotate_cmd)
        rospy.loginfo("Finished 180 degree rotation")

    def get_yaw_from_orientation(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def get_continuous_yaw(self, orientation):
        yaw = self.get_yaw_from_orientation(orientation)
        return yaw + self.yaw_offset

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def run(self):
        rospy.loginfo("start run")
        while not rospy.is_shutdown():
            if self.run_loop:
                if not self.has_moved_forward:
                    self.move_forward(-1)  # Move forward by 0.5 meters
                    self.has_moved_forward = True

                if self.has_moved_forward and not self.has_rotated:
                    self.rotate_180()
                    self.has_rotated = True

                if self.has_rotated and not self.has_moved_backward:
                    self.move_forward(-1)  # Move backward by 1.8 meters
                    self.has_moved_backward = True

                if self.has_moved_backward:
                    rospy.loginfo("Completed movement sequence")
                    break  # Exit the loop after completing the sequence

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

