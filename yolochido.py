#!/usr/bin/env python3  

from ultralytics import YOLO  
import rclpy  
from rclpy.node import Node  
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge  
import cv2  
import numpy as np   
from yolo_msg.msg import InferenceResult  
from yolo_msg.msg import Yolov8Inference  
from std_msgs.msg import String
  
# video_source/raw
#image_raw

class CameraSubscriber(Node):  

    def __init__(self):  

        super().__init__('camera_subscriber')  

        self.model = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/traffic_signals.pt')  
        self.model2 = YOLO('~/ros2_ws/src/yolov8_ros2/yolov8_ros2/traffic_lights.pt')

        self.yolov8_inference = Yolov8Inference()
        self.yolov8_inference2 = Yolov8Inference()   

        self.bridge = CvBridge()  

        self.subscription = self.create_subscription(Image, 'video_source/raw', self.camera_callback, 10)  

        self.image_width = 640 

        self.image_height = 480 

        self.img = np.ndarray((self.image_height, self.image_width, 3))  

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1) 
        self.yolov8_pub2 = self.create_publisher(Yolov8Inference, "/Yolov8_Inference2", 1)  

        self.img_pub = self.create_publisher(Image, "/inference_result", 1)  

        self.class_name_pub = self.create_publisher(String, "detected_class_name", 10)
        self.class_name_pub2 = self.create_publisher(String, "detected_class_name2", 10)

        

        timer_period = 1.0  # seconds  

        self.timer = self.create_timer(timer_period, self.timer_callback)  
        self.timer2 = self.create_timer(0.5, self.timer_callback2)  


    def timer_callback2(self):
        results2= self.model2(self.img)

        self.yolov8_inference2.header.frame_id = "inference"  

        self.yolov8_inference2.header.stamp = self.get_clock().now().to_msg()
        for r in results2:  

            boxes = r.boxes  

            for box in boxes:  

                self.inference_result2 = InferenceResult()  

                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format  

                c = box.cls  

                class_name2 = self.model2.names[int(c)]  
                self.class_name_pub2.publish(String(data=class_name2))

                self.inference_result2.top = int(b[0])  

                self.inference_result2.left = int(b[1])  

                self.inference_result2.bottom = int(b[2])  

                self.inference_result2.right = int(b[3])  

                self.yolov8_inference.yolov8_inference.append(self.inference_result2)  

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")
            
            annotated_frame = results2[0].plot()  

            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")    

            self.img_pub.publish(img_msg) 
             
            self.yolov8_pub2.publish(self.yolov8_inference2)  

            self.yolov8_inference2.yolov8_inference.clear() 


    def timer_callback(self):  

        results = self.model(self.img)  
        

        self.yolov8_inference.header.frame_id = "inference"  

        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()  

        for r in results:  

            boxes = r.boxes  

            for box in boxes:  

                self.inference_result = InferenceResult()  

                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format  

                c = box.cls  

                class_name = self.model.names[int(c)]  
                self.class_name_pub.publish(String(data=class_name))

                self.inference_result.top = int(b[0])  

                self.inference_result.left = int(b[1])  

                self.inference_result.bottom = int(b[2])  

                self.inference_result.right = int(b[3])  

                self.yolov8_inference.yolov8_inference.append(self.inference_result)  

            #camera_subscriber.get_logger().info(f"{self.yolov8_inference}")  

        #annotated_frame = results[0].plot()  

        #img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")    

        #self.img_pub.publish(img_msg)  

        self.yolov8_pub.publish(self.yolov8_inference)  

        self.yolov8_inference.yolov8_inference.clear()  

  

    def camera_callback(self, msg):  
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")  

        # print(img.shape)  

        scale = 2

        w = int(self.image_width/scale)  

        h = int(self.image_height/scale)  

        small = (w, h)  

        self.img = cv2.resize(img, small)  
  

def main(args=None):  

    rclpy.init(args=args)  

    camera_subscriber = CameraSubscriber()  

    rclpy.spin(camera_subscriber)  

    rclpy.shutdown()  

  

  

if __name__ == '__main__':  

    main()
