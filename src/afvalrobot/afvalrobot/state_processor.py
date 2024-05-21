import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
import time
CLOSING_DISTANCE = 5

class state_processor(Node):

    def __init__(self):
        super().__init__(type(self).__name__) #give this node the name of the class
        self.bot_state=0
        self.camera_sub = self.create_subscription(
            String,                 #msg type
            'cameraState',         #topic name
            self.camera_callback,  #subscriber callback function
            1                      #queue size
        )
        self.trashDist = self.create_subscription(
            Float32,                 #msg type
            'trashDistance',         #topic name
            self.trashDistCallback,  #subscriber callback function
            1                      #queue size
        )
        self.gripper_sub = self.create_subscription( #gripper
            Int32,                 #msg type
            'gripperState',         #topic name
            self.gripper_callback,  #subscriber callback function
            1                      #queue size
        )
        self.sonar_sub = self.create_subscription( #gripper
            Float32,                 #msg type
            'distance',         #topic name
            self.sonar_callback,  #subscriber callback function
            1                      #queue size
        )
        self.state = self.create_publisher(
            Int32,      #msg type
            'currentState', #topic name 
            1        #queue size
        )
        self.timer = self.create_timer(1.0, self.initPublish)
        self.bot_state = 0 #initial state
        self.prev = 0
        self.dist2Trash = 3.0 #Value that is larger than 1.5
        self.initFlag = True
    def gripper_callback(self,msg):
        self.gripperState = msg.data
        if self.bot_state == 1:
            # als van 1 naar 0 (van open naar dicht) dan sate op 2
            if self.gripperState == 1: # gripper is open
                self.prev=1
            elif self.gripperState == 0 and self.prev == 1:
                self.prev = 0
                self.bot_state = 2
                self.publish_state()
                self.get_logger().info('From gripper: currentState change 1->2:')
        elif self.bot_state == 3 and self.gripperState == 1:         
            # gripper is open
            self.bot_state = 4
            self.publish_state()
            self.get_logger().info('From gripper: currentState change 3->4:')

        #self.publish_state()

 
    def camera_callback(self, msg):
        self.cameraState = msg.data            
        if self.dist2Trash == 1.5 and self.bot_state == 2:
            self.bot_state = 3
            self.publish_state()
            self.get_logger().info('From camera: currentState change 2->3:')

        elif self.cameraState != 'not' and self.bot_state == 0:
            self.bot_state = 1
            self.publish_state()
            self.get_logger().info('From camera: currentState change 0->1:')

        

    def sonar_callback(self,msg):
        self.sonarDist = msg.data
        if self.bot_state == 4 and self.sonarDist > 3*CLOSING_DISTANCE:
            time.sleep(5)
            self.bot_state = 0
            self.publish_state()
            self.get_logger().info('From camera: currentState change 4->0:')
        
    def trashDistCallback(self, msg):
        self.dist2Trash = msg.data

    def initPublish(self):
        if self.initFlag == True:
            self.publish_state()
        self.initFlag = False
    def publish_state(self):
        msg = Int32()
        msg.data = self.bot_state
        self.state.publish(msg)
        self.get_logger().info('I published state: %s' % msg.data)
                
    def dummy():
        pass

    
           

def main(args=None):
    rclpy.init(args=args)
    #make new publisher object
    node = state_processor()
    #keep node alive
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
