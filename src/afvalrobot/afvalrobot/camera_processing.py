import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from math import sqrt
from math import tan
from std_msgs.msg import String, Int32

class CameraProcessing(Node):
    def __init__(self):
        super().__init__('cameraprocessing')
        
        self.currentState = 0
        self.stateSubscription = self.create_subscription(Int32, 'currentState', self.state_callback, 1)
        self.subscription # Prevent unused variable warning

        self.publisher_ = self.create_publisher(String, 'cameraState', 1)
        self.camera = cv2.VideoCapture(0)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.cm = np.array([[823.93985557, 0., 322.76228491], [0., 825.11141958, 279.6240493], [0., 0., 1.]]) # Correction for cameralens
        self.parameters = cv2.aruco.DetectorParameters()
        self.ms = 0.1
        self.trash = [0,1]
        self.trashcans = [2, 3]
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
                if id not in self.trash | id not in self.trashcans: # None valuable IDs 
                    continue
                rvec, tvec, rejimp= self.estimatePoseSingleMarkers(corners[i]) # Corners of marker on screen are input, translation and rotation of aruco are output
                if len(tvec)!=0: 
                    distance = sqrt(tvec[0][0]**2+tvec[0][2]**2) # Translation vector is used to get position of aruco
                    h = tan(tvec[0][0]/tvec[0][2])       
                    return distance, h, 1, id
        return 1.5,0,0,-1

    def timer_callback(self):
        self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
        ret, frame = self.camera.read() # Read single frame from camera
        if not ret:
            self.get_logger().info('Failed to read frame from camera')
            return
        var1, var2, var3, idfound= self.poseEstimation(frame)
        msg = {}
        if (currentState == 0) & (id in trash):
            msg.data = "Thrash found"
        elif (currentState == 1) & (id in trash):
            msg.data = "Location trash:" + str(var1) + ";" + str(var2) + ";" + str(var3)
        elif (currentState == 2) & (id in trashcans):
            msg.data = "Trashcan found"
        else:
            msg.data = ""
        msg.data = idfound.
        self.publisher.publish(msg)
    
    def state_callback(self, msg):
        self.currentState = msg.data
        self.get_logger().info('I heard state: %s' % msg.data)
        # Modify depending on states
        if self.currentState == 1:
            timer_callback()

def main(args = None):
    rclpy.init(args = args)
    cp = CameraProcessing()
    rclpy.spin(cp)
    vc.camera.release() # Release camera
    rclpy.shutdown()

if __name__ == '__main__':
    main()