import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class PubSubClass(Node):

    def __init__(self):
        super().__init__('pub_sub_with_classes')
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.sub = self.create_subscription(String, "detected_class_name", self.listener_callback, 10)
        self.flag_subscription = self.create_subscription(String, 'crossing_detected', self.flag_callback, 10)
        self.continue_pub = self.create_publisher(String, 'continue_following', 10)
        timer_period = 0.1  # Check the state every 0.1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Node initialized!!!")
        self.vel = Twist()
        self.last_detected_signal = "nothing"
        self.action_start_time = None
        self.action_duration = 0
        self.detect_signal = False

    def timer_callback(self):
        if self.detect_signal == True:

            if self.last_detected_signal == "Forward":
                self.get_logger().info("moving forward")
                self.vel.linear.x = 0.15
                self.vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(2.5)
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.0                   
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(0.5)
                self.continue_pub.publish(String(data="continue"))
                self.detect_signal = False 

            elif self.last_detected_signal == "Turn_right":
                self.get_logger().info("turning right")
                self.vel.linear.x = 0.15
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(2.3)
                self.vel.linear.x = 0.0
                self.vel.angular.z = -0.5
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(3)
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.15                    
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(1)
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.0                   
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(0.5)
                self.continue_pub.publish(String(data="continue"))
                self.detect_signal = False 

            elif self.last_detected_signal == "Turn_left":
                self.get_logger().info("turning left")
                self.vel.linear.x = 0.15
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(2.3)
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.5
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(3)
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.15                    
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(1)
                self.vel.angular.z = 0.0
                self.vel.linear.x = 0.0                   
                self.cmd_vel_pub.publish(self.vel)
                time.sleep(0.5)
                self.continue_pub.publish(String(data="continue"))
                self.detect_signal = False 

    def listener_callback(self, msg):
        self.last_detected_signal = msg.data  # msg.data is the string contained inside the ROS message
        self.get_logger().info(f"Last detected signal: {self.last_detected_signal}")

    def flag_callback(self, msg):
        if msg.data == "pedestrian crossing detected":
            self.get_logger().info("pedestrian crossing detected")
            self.detect_signal = True
            #self.action_start_time = self.get_clock().now().seconds_nanoseconds()[0]

            #if self.last_detected_signal == "Forward" or self.last_detected_signal == "turn_left" or self.last_detected_signal == "turn_right":
                #self.action_duration = 6.0  # Execute action for 3.5 seconds

        
def main(args=None):
    rclpy.init(args=args)
    m_p = PubSubClass()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
