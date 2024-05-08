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
		self.distance = 20
		self.stateSubscription = self.create_subscription(Int32, 'currentState', self.state_callback, 1)		   
		self.wheelsPublisher = self.create_publisher(Twist, '/cmd_vel', 1) 
		self.distanceSubcription = self.create_subscription(Float32, 'distance', self.distance_callback, 1)
	
	def stop(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.0, 0.0
		self.wheelsPublisher.publish(cmd)

	def search(self):
		# drive forward 5 seconds
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = -0.1, 0.0
		self.wheelsPublisher.publish(cmd)
		
		start = time.time()		
		while(self.distance > 10 and self.currentState == 0 and time.time() - start < 5):
			pass

		#self.stop() # stop driving
		#time.sleep(1)
		
		if self.currentState != 0: 
			return
		
		cmd.linear.x, cmd.angular.z = 0.0, 1.0 
		self.wheelsPublisher.publish(cmd)
		start = time.time()
		# rotate for 3 seconds
		while(self.currentState == 0 and time.time() - start < 3):
			self.state_callback()
			pass


	def get_object(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = -0.1, 0.0
		self.wheelsPublisher.publish(cmd)
		# Drive to object
		while (self.distance > 5 and self.currentState == 1):
			self.get_logger().info('I heard sate in while loop: "%s"' % self.currentState)
			self.state_callback()
			time.sleep(1)
			pass
		# Stop
		self.stop()

	def controlWheels(self, state):
		
		# Search object
		if state == 0:
			self.search()

		# Found object
		elif state == 1:
			self.get_object()

		# Stop
		elif state == 2:
			self.stop()
		
	def state_callback(self, msg):
		self.currentState = msg.data
		self.get_logger().info('I heard state: "%s"' % msg.data)
		# Modify depending on states
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
