import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
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
		self.cameraSubcription = self.create_subscription(String, 'cameraState', self.camera_callback, 1)
		self.counter = 0

	def stop(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.0, 0.0
		self.wheelsPublisher.publish(cmd)

	def start(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = -0.1, 0.0
		self.wheelsPublisher.publish(cmd)

	def start_slow(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = -0.085, 0.0
		self.wheelsPublisher.publish(cmd)
	
	def back(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.1, 0.0
		self.wheelsPublisher.publish(cmd)

	def rotate_left(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.0, 0.5
		self.wheelsPublisher.publish(cmd)

	def rotate_left_slow(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.0, 0.1
		self.wheelsPublisher.publish(cmd)
	
	def rotate_right_slow(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.0, -0.1
		self.wheelsPublisher.publish(cmd)

	def rotate_right(self):
		cmd = Twist()
		cmd.linear.x, cmd.angular.z = 0.0, -0.5
		self.wheelsPublisher.publish(cmd)
		
	def state_callback(self, msg):
		self.currentState = msg.data
		self.get_logger().info('I heard state: "%s"' % msg.data)

		# state 0: search object
		if self.currentState == 0:
			self.start()
			
		# state 1: drive to object
		elif self.currentState == 1:
			self.start_slow()

		# state 2: drive to trash can
		elif self.currentState == 2:
			self.back()
			self.get_logger().info('I go back')
			time.sleep(1)
		elif self.currentState == 3:
			self.stop()
		elif self.currentState == 4:
			self.back()
			time.sleep(3)
			self.rotate_left()
			time.sleep(2)
			

	def distance_callback(self, msg):
		self.distance = msg.data
		self.get_logger().info('I heard distance: "%s"' % msg.data)

		# state 0: search object -> close to wall rotate 
		if (self.currentState == 0 and self.distance < 20):
			self.rotate_left()
			time.sleep(2)
			self.start()
		
		# state 1: found object -> drive straight to object
		elif (self.currentState == 1 and self.distance < 4):
			self.stop()
			time.sleep(1)


	def camera_callback(self, msg):
		self.camera = msg.data
		self.get_logger().info('I heard direction: "%s"' % msg.data)

		if self.currentState == 1 or self.currentState == 2: 
			if self.camera == "left":
				self.rotate_left_slow()

			elif  self.camera == "right":
				self.rotate_right_slow()
			
			elif  self.camera == "middle":
				self.start_slow()
			
			elif self.camera == "not":
				self.counter += 1
				self.get_logger().info('I count not' + counter)

			# find trash
			if self.counter >= 5 and self.currentState == 1:
				self.counter = 0
				self.rotate_left_slow()	
			# find aruco
			elif self.counter >= 3 and self.currentState == 2:
				self.counter = 0
				self.get_logger().info('I rotate')
				self.rotate_left_slow()	
			
	
def main(args=None):
	rclpy.init(args=args)
	wheels = Wheels()

	rclpy.spin(wheels)

	wheels.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
