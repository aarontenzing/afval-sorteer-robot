import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from math import sqrt
from math import tan
from std_msgs.msg import String, Int32, Float32

class CameraProcessing(Node):
    def __init__(self):
        super().__init__('cameraprocessing')
        
        self.currentState = 0
        self.stateSubscription = self.create_subscription(Int32, 'currentState', self.state_callback, 1)


        self.zoek="cola"
        self.publisher_ = self.create_publisher(String, 'cameraState', 1)
        self.publisher_trashDistance = self.create_publisher(Float32, 'trashDistance', 1)
        self.camera = cv2.VideoCapture(0)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.cm = np.array([[823.93985557, 0., 322.76228491], [0., 825.11141958, 279.6240493], [0., 0., 1.]]) # Correction for cameralens
        self.parameters = cv2.aruco.DetectorParameters()
        self.ms = 0.1
        self.trashcans = [1, 3]
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.dm = np.array([[6.29137073e-02, -7.33484417e-01, 6.53444356e-03, 3.83894903e-03, 1.16325776e+01]])
        self.timer = self.create_timer(0.3, self.timer_callback)

    def estimatePoseSingleMarkers(self, corners): # Replaces depricated function
        marker_size = self.ms
        marker_points = np.array([[-marker_size / 2,  marker_size / 2, 0],
                                  [ marker_size / 2,  marker_size / 2, 0],
                                  [ marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        i = 0
        for c in corners:
            empty, R, t = cv2.solvePnP(marker_points, corners[i], self.cm, self.dm, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R) # Rotation
            tvecs.append(t) # Translation
            trash.append(empty) # ?
        return rvecs, tvecs, trash

    def poseEstimation(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = self.detector.detectMarkers(gray)
        if len(corners) > 0:    
            for i, id in enumerate(ids):
                if id not in self.trashcans: # None valuable IDs 
                    continue
                rvec, tvec, rejimp= self.estimatePoseSingleMarkers(corners[i]) # Corners of marker on screen are input, translation and rotation of aruco are output
                if len(tvec)!=0: 
                    distance = sqrt(tvec[0][0]**2+tvec[0][2]**2) # Translation vector is used to get position of aruco
                    h = tan(tvec[0][0]/tvec[0][2])       
                    return distance, h, 1, id
        return 1.5,0,0,-1


    def object_detect(self):
        self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ret, frame = self.camera.read() 
        if not ret:
            self.get_logger().info('Failed to read frame from camera')
            return
        msg = String()
        obj=self.detect_cola_can(frame)
        msg.data = obj
        self.publisher_.publish(msg)
        self.get_logger().info(obj)

        return


    def trashcan_detect(self):
        self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ret, frame = self.camera.read() # Read single frame from camera
        if not ret:
            self.get_logger().info('Failed to read frame from camera')
            return
        distance, angle, var3, idfound = self.poseEstimation(frame)
        msg = String()
        msg_dist = Float32()

        # id = 1 -> obj "cola" and id = 3 -> obj "1"
        if idfound == -1 or ((self.zoek == 'cola' and idfound == 1) or (self.zoek == "1" and idfound == 3)):
            msg.data = "not"
        else:
            if angle < -0.2:
                msg.data = "left"
            elif angle > 0.2:
                msg.data = "right"
            else:
                msg.data = "middle"
            msg_dist.data = distance
            self.publisher_.publish(msg)
            self.publisher_trashDistance.publish(msg)
        
        #msg.data = "Location trashcan id " + str(idfound) + ":" + str(var1) + ";" + str(var2) + ";" + str(var3)
        #self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.get_logger().info("distance: %s" % msg_dist.data)
    
    def state_callback(self, msg):
        self.currentState = msg.data
        self.get_logger().info('I heard state: %s' % msg.data)



    def timer_callback(self):
        if self.currentState == 0 or self.currentState == 1 :
            self.object_detect()
        else:
            #self.get_logger().info('I execute vuilback_detect')
            self.trashcan_detect()

    def detect_cola_can(self,image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
    

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        image_center_x = image.shape[1] / 2
        middle_tolerance = image.shape[1] * 0.10 
        min_area_threshold = 200

    
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) < min_area_threshold:
                return "not"
            else:
                M = cv2.moments(largest_contour)
                self.zoek="0"
                if M["m00"] != 0:

                    cx = int(M["m10"] / M["m00"])
                    if cx < image_center_x - middle_tolerance:
                        return "left"
                    elif cx > image_center_x + middle_tolerance:
                        return "right"
                    else:
                        return "middle"
        else:
            return "not"

def main(args = None):
    rclpy.init(args = args)
    cp = CameraProcessing()
    rclpy.spin(cp)
    vc.camera.release() # Release camera
    rclpy.shutdown()

if __name__ == '__main__':
    main()