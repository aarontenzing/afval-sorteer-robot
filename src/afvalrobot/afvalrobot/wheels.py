import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class Wheels(Node):

	def __init__(self):
		super().__init__('wheels')
		self.currentState = 0
		self.stateSubscription = self.create_subscription(Int32, 'currentState', self.state_callback, 1)		   
		self.wheelsPublisher = self.create_publisher(Twist, '/cmd_vel', 1) 
	
	def controlWheels(self, state):
		if state == 0:
			linear, angular = 0.0, 0.0
		# STRAIGHT
		elif state == 1:
			linear, angular = 0.22, 0.0
		# RIGHT
		elif state == 2:
			linear, angular = 0.0, -1.0
		# LEFT
		elif state == 3:
			linear, angular = 0.0, 1.0
		# BACK 
		elif state == 4:
			linear, angular = -0.22, 0.0
		else:
			linear, angular = 0.0, 0.0
		
		cmd = Twist()
		cmd.linear.x =	linear
		cmd.angular.z = angular

		self.wheelsPublisher.publish(cmd)
			
		
	def state_callback(self, msg):
		self.currentState = msg.data
		self.get_logger().info('I heard state: "%s"' % msg.data)

		#Modify depending on states
		self.controlWheels(self.currentState) 
	
def main(args=None):
	rclpy.init(args=args)
	wheels = Wheels()

	rclpy.spin(wheels)

	wheels.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
