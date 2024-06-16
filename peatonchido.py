import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


banderaStop=False
banderaGive=False
banderaworkes=False
nolinedetecterNormal=True
Calculo=True

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower_node')              
        self.bridge = CvBridge()
        self.sub_class = self.create_subscription(String, "detected_class_name", self.listener_callback, 10)
        self.sub_class2 = self.create_subscription(String, "detected_class_name2", self.listener_callback2, 10)
        self.sub = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)
        #self.pub_line = self.create_publisher(Image, 'line_follower', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.crossing_pub = self.create_publisher(String, 'crossing_detected', 10)
        self.continue_sub = self.create_subscription(String, 'continue_following', self.continue_callback, 10)
        self.robot_vel = Twist()
        self.image_received_flag = False
        self.last_detected_signal = "nothing"
        self.last_detected_signal2 = "nothing"
        self.pause_flag = False
        self.workers_ahead_start_time = None
        dt = 0.1
        self.timer = self.create_timer(dt, self.timer_callback)
        self.get_logger().info('Line Follower Node started')

    def camera_callback(self, msg):
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received_flag = True
        except Exception as e:
            self.get_logger().info('Failed to get an image: {}'.format(str(e)))

    def detect_pedestrian_crossing(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        row_sum = np.sum(binary, axis=1)
        black_pixel_count = np.count_nonzero(row_sum < 255 * binary.shape[1] * 0.5)
        return black_pixel_count > 10  #10

    def timer_callback(self):
        global banderaStop, banderaworkes, banderaGive,nolinedetecterNormal, Calculo

        if not self.image_received_flag or self.pause_flag:
            return

        image = self.cv_img.copy()
        height, width = image.shape[:2]
      
        #right_roi = image[0:roi_height, width - roi_width:width]
        line_image = image[161:240, 0:320] 
        gray = cv2.cvtColor(line_image, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
        if self.detect_pedestrian_crossing(line_image):
            self.get_logger().info('Pedestrian Crossing Detected') 
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0
            self.cmd_vel_pub.publish(self.robot_vel)
            time.sleep(0.2)
            if self.last_detected_signal2 == "Green":
                self.get_logger().info('Green Color Detected')
                self.crossing_pub.publish(String(data="pedestrian crossing detected"))
                self.pause_flag = True
            if self.last_detected_signal2 == "Red":
                self.get_logger().info('Red Color Detected')
            if self.last_detected_signal2 == "Yellow":
                self.get_logger().info('Yellow Color Detected')

        else:
            if nolinedetecterNormal==True:
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    M = cv2.moments(largest_contour)
                    if M['m00'] != 0:
                        
                            self.get_logger().info('Line Detected')
                            cx = int(M['m10'] / M['m00'])
                            cy = int(M['m01'] / M['m00'])

                            mark_size = 10
                            if  cx > 60 and cx < 260 : #cx > 85 and cx < 235
                                cv2.line(line_image, (cx - mark_size, cy - mark_size), (cx + mark_size, cy + mark_size), (0, 0, 255), 2)
                                cv2.line(line_image, (cx + mark_size, cy - mark_size), (cx - mark_size, cy + mark_size), (0, 0, 255), 2)

                            
                                error = cx - width // 2
                                self.robot_vel.angular.z = (-float(error) / 100) * 0.5
                                print("Valor en Z ", self.robot_vel.angular.z)
                                self.robot_vel.linear.x = 0.08
                            
                            if self.last_detected_signal == "Workers_ahead":
                                self.get_logger().info('Line Detected with Workers Ahead')
                                if banderaworkes == False:
                                    if self.workers_ahead_start_time is None:
                                        self.workers_ahead_start_time = time.time()
                                    elapsed_time = time.time() - self.workers_ahead_start_time
                                    if elapsed_time < 5:
                                        self.robot_vel.linear.x *= 0.5
                                    else:
                                        self.workers_ahead_start_time = None
                                        self.last_detected_signal = "nothing"
                                        banderaworkes = True      

                            elif self.last_detected_signal == "Give_way":
                                self.get_logger().info('Line Detected with Give Way')
                                if banderaGive == False:
                                    if self.workers_ahead_start_time is None:
                                        self.workers_ahead_start_time = time.time()
                                    elapsed_time = time.time() - self.workers_ahead_start_time
                                    if elapsed_time < 2:
                                        self.robot_vel.linear.x *= 0.5
                                    else:
                                        self.workers_ahead_start_time = None
                                        self.last_detected_signal = "nothing"
                                        banderaGive = True

                            elif self.last_detected_signal == "Stop":
                                if banderaStop==False:
                                    self.get_logger().info("Stop Signal Detected")
                                    self.robot_vel.linear.x = 0.0
                                    self.robot_vel.angular.z = 0.0
                                    self.cmd_vel_pub.publish(self.robot_vel)
                                    time.sleep(10)
                                
                                    self.last_detected_signal = "nothing"
                                    banderaStop=True
                                    banderaworkes=False
                                    banderaGive =False     

                            elif self.last_detected_signal2 == "Yellow":
                                
                                self.get_logger().info('Yellow Color Detected')
                                self.robot_vel.linear.x *= 0.5
                                self.robot_vel.angular.z *= 0.5
                                                          

                            self.cmd_vel_pub.publish(self.robot_vel)
                            
    
                else:
                    self.get_logger().info('No line detected')
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0
            
            #**************************************---------------------------------------
         

        

        #self.pub_line.publish(self.bridge.cv2_to_imgmsg(line_image, 'bgr8'))
      
        

    def continue_callback(self, msg):
        if msg.data == "continue":
            self.pause_flag = False
            self.get_logger().info("Continuing line following")

    def listener_callback(self, msg):
        self.last_detected_signal = msg.data  # msg.data is the string contained inside the ROS message
        #print("se recicbio esta senial ",self.last_detected_signal)

    def listener_callback2(self, msg):
        self.last_detected_signal2 = msg.data 
        print("se recicbio esta luz ",self.last_detected_signal2)   

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


