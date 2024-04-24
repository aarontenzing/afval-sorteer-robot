import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
import time

# MAX linear speed = 0.22 // m/s
# MAX angular speed = 2.84 // rad/s

class Wheels(Node):

	def __init__(self):
		super().__init__('wheels')
		self.currentState = 0
		self.stateSubscription = self.create_subscription(Int32, 'currentState', self.state_callback, 1)		   
		self.wheelsPublisher = self.create_publisher(Twist, '/cmd_vel', 1) 
		self.distanceSubcription = self.create_subscription(Float32, 'distance', self.distance_callback, 1)
	
	def search(self):
		cmd = Twist()
		while(self.distance_callback > 10):
			# drive forward 5 seconds
			cmd.linear.x, cmd.angular.z = 0.1, 0.0
			self.wheelsPublisher.publish(cmd)
			time.sleep(5)
			# turn for 1 second
			cmd.linear.x, cmd.angular.z = 0.0, 1.0
			self.wheelsPublisher.publish(cmd)
			time.sleep(1)
		
		# turn for 3 second 
		cmd.linear.x, cmd.angular.z = 0.0, 1.0
		self.wheelsPublisher.publish(cmd)
		time.sleep(3)

	def controlWheels(self, state):
		
		# Search object
		if state == 0:
			while (1):
				self.search()

		# FORWARD
		elif state == 1:
			linear, angular = -0.1, 0.0
		# RIGHT
		elif state == 2:
			linear, angular = 0.0, -2.5
		# LEFT
		elif state == 3:
			linear, angular = 0.0, 2.55
		# BACK 
		elif state == 4:
			linear, angular = 0.1, 0.0
		else:
			linear, angular = 0.0, 0.0
			
		
	def state_callback(self, msg):
		self.currentState = msg.data
		self.get_logger().info('I heard state: "%s"' % msg.data)

		#Modify depending on states
		self.controlWheels(self.currentState) 

	def distance_callback(self, msg):
		self.distance = msg.data
		self.get_logger().info('I heard distance: "%s"' % msg.data)
	
def main(args=None):
	rclpy.init(args=args)
	wheels = Wheels()

	rclpy.spin(wheels)

	wheels.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
