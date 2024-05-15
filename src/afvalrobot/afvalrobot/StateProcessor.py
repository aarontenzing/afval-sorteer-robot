import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32

CLOSING_DISTANCE = 5

class StateProcessor(Node):

    def __init__(self):
        super().__init__(type(self).__name__) #give this node the name of the class
        self.bot_state=0
        self.camera_sub = self.create_subscription(
            String,                 #msg type
            'cameraState',         #topic name
            self.camera_callback,  #subscriber callback function
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
        self.timer = self.create_timer(0.3, self.process_state)
        self.bot_state = 0 #initial state
        self.prev = 0

    def gripper_callback(self,msg):
        self.gripperState = msg.data
        if self.bot_state == 1:
            # als van 1 naar 0 (van open naar dicht) dan sate op 2
            if self.gripperState == 1: # gripper is open
                self.prev=1
            elif self.gripperState == 0 and self.prev == 1:
                self.prev = 0
                self.bot_state == 2
        elif self.bot_state == 3 and self.gripperState == 1:         
            # gripper is open
            self.bot_state == 4
        self.publish_state()

 
    def camera_callback(self, msg):
        self.cameraState = msg.data
        id = self.cameraState.split(':')[0].strip()
        var = self.cameraStateplit(':')[1].strip().split(';')
        distance = var[0]
        h = var[1]
        # var[2] is usually == 1 if nothing is detected then 0
        if self.bot_state == 2:
            if id == -1:
                self.bot_state = 3
        
        self.publish_state()

    def sonar_callback(self,msg):
        self.sonarDist = msg.data
        if self.bot_state == 4 and self.sonarDist > 3*CLOSING_DISTANCE:
            self.bot_state == 0

        self.publish_state()

    def publish_state(self):
        msg = Int32()
        msg.data = self.bot_state
        self.state_publisher.publish(msg)
                
    def dummy():
        pass

    
           

def main(args=None):
    rclpy.init(args=args)
    #make new publisher object
    node = StateProcessor()
    #keep node alive
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
