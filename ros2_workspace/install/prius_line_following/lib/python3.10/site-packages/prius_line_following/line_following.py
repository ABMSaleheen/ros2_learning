import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Line_Detection_And_Following(Node):
    def __init__(self):
        super().__init__('prius_line_follower') #Node Name

        self.subscriber = self.create_subscription(Image, '/camera/image_raw', self.process_data, 10) #Create Subscriber     
        self.bridge = CvBridge()  # converting ros images to opencv data

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 40) # Periodic Publisher call
        timer_period = 0.2; self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity = Twist()
        self.error = 0.0
        self.action= ""
        self.prev_edge = []

    ## Subscriber callback function
    def process_data(self,data):
    # performing conversion
        frame = self.bridge.imgmsg_to_cv2(data)

        ######## Segmentation
        light = 30
        dark = 210

        ## Defining the color Upper bound and Lower bound limits to extract 
        light_line = np.array([light,light,light])
        dark_line = np.array([dark,dark,dark])

        mask = cv2.inRange(frame, light_line, dark_line)

        ###-----------------------------# Boundaries Extraction----------------------
        ## applying the canny edge detector function
        canny = cv2.Canny(mask, 40, 10)

        img = canny

        # print("Original Frame_Size= ",img.shape) 

        ### ----# Cropping
        r1 = 150; c1=0
        cropped_ht = 200
        cropped_wdth = 640
        img = canny[r1: r1+ cropped_ht, c1: c1+ cropped_wdth]

        row_of_intrst = 140

        edge=[0]
        


        
        for i in range (639):
            if(img[row_of_intrst,i] == 255):
                if abs(edge[-1] - i) < 5 :
                    edge.pop()         
                edge.append(i)

            # for j in range(0, cropped_ht)
                
        edge = edge[1:] # Remove the 0

        if(len(edge) < 1):
            self.action = "No edge ---------"

        if (len(edge) < 2):
            print("Tracking Dummy (prev edge)")
            edge = self.prev_edge


            # self.prev_edge.clear()

                # print("prev edge",self.prev_edge)
                # print("edge", edge)

                
            

        #         ## We only need two points # one from left line # second from right line 
        # # When values are 4
        # if(len(edge)==4):
        #     edge[0]=edge[0]
        #     edge[1]=edge[2]
        # ## When Values are 3 and if they are 2 we donot need to process them
        # ## If the values of pixels is greater then 5 then they are adjecent to eachother
        # if(len(edge)==3):
        #     for i in range(len(edge)):
        #         if(edge[1]-edge[0] > 5): ## meaning idx(0) and idx(1) are ok [193, 506, 507 ]
        #             edge[0]=edge[0]
        #             edge[1]=edge[1]
        #         else:#[193, 194, 507 ]
        #             edge[0]=edge[0]
        #             edge[1]=edge[2]



        print("edge to track",edge)

        self.prev_edge = edge[0:2]
        print("prev edge", self.prev_edge)

        dist_betweeen_edge_pts = (edge[1] - edge[0])
        mid_point = +edge[0] + (dist_betweeen_edge_pts/2)  # Mid point of 2 edge pts
        img[row_of_intrst, int(mid_point)] = 255  # applying a white pixel to Edges' mid pt

        frame_mid_pt = int(639/2)
        # applying white to Frame mid pt------
        img[row_of_intrst, frame_mid_pt] = 255 
        img[row_of_intrst-1, frame_mid_pt] = 255 # Frame mid pt
        img[row_of_intrst+1, frame_mid_pt] = 255 # Frame mid pt       


        self.error = frame_mid_pt - mid_point

        f_image = cv2.putText(img, self.action, (100,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,), 2, cv2.LINE_AA)

        #write the frames to a vid
        # self.out.write(frame)

        # displaying what is being recorded 
        cv2.imshow('output image',f_image)
        cv2.waitKey(1)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # publisher callback function
    def send_cmd_vel(self):
        # if not self.prev_edge:
        if self.action == "No edge ---------":
            self.velocity.angular.z = 0.0
            self.velocity.linear.x = 0.0
            self.action = "Stopped"

        else:
            self.velocity.linear.x = 1.0

            if (self.error > 0): ## Go left
                self.velocity.angular.z = 0.25
                self.action = "Steering Left"
            
            if (self.error < 0):
                self.velocity.angular.z = -0.25
                self.action = "Steering Right"



        self.publisher.publish(self.velocity)




def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Line_Detection_And_Following()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

