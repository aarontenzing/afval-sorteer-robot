import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32

class StateProcessor(Node):

    def __init__(self):
        super().__init__(type(self).__name__) #give this node the name of the class
        self.bot_state=0
        self.camera_sub = self.create_subscription(
            String,                 #msg type
            '/object',         #topic name
            self.camera_callback,  #subscriber callback function
            1                      #queue size
        )
        self.gripper_sub = self.create_subscription( #gripper
            Int32,                 #msg type
            'gripperState',         #topic name
            self.gripper_callback,  #subscriber callback function
            1                      #queue size
        )
        self.state = self.create_publisher(
            Int32,      #msg type
            '/state', #topic name 
            1        #queue size
        )
        self.timer = self.create_timer(0.3, self.process_state)
        self.bot_state = 0 #initial state
        self.prev = 0

    def gripper_callback(self,msg):
        self.gripperState = msg.data
        # als van 1 naar 0 dan sate op 2
        if self.bot_state == 1:
            if self.gripperState == 1:
                self.prev=1
            elif self.gripperState == 0 and self.prev == 1:
                self.bot_state == 2
        self.publish_state()

 
    def camera_callback(self, msg):
        self.object = msg.data
        if self.boy_state == 0:
            if self.object == "gevonden":
                self.bot_state = 1
        
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
