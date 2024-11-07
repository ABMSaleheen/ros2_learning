import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Video_get(Node):
    def __init__(self):
        super().__init__('video_subscriber') #Node Name

        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10) #Create Subscriber

        ## setting for writing the frames into a video
        self.out = cv2.VideoWriter('/home/saleheen_linux/others/cv_vid/output.avi', 
                                   cv2.VideoWriter.fourcc('M','J','P','G'),10,(640,480)) 
        
        self.bridge = CvBridge()  # converting ros images to opencv data

      ## Subscriber callback function
    def process_data(self,data):
    # performing conversion
        frame = self.bridge.imgmsg_to_cv2(data)

        #write the frames to a vid
        self.out.write(frame)
        # displaying what is being recorded 
        cv2.imshow("output", frame)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Video_get()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

