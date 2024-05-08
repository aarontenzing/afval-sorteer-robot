import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32, Int32

CLAW = 13
CLOSING_DISTANCE = 5


GPIO.setmode(GPIO.BCM) #BCM is GPIO number
GPIO.setup(CLAW, GPIO.OUT)
claw_pwm = GPIO.PWM(CLAW,50)
claw_pwm.start(0)

class Gripper(Node):

	def __init__(self):
		super().__init__('gripper')
		self.currentState = 0
		self.distSubscription = self.create_subscription(
			Float32,
			'distance',
			self.distance_callback,
			1)
		self.stateSubscription = self.create_subscription(
			Int32,
			'currentState',
			self.state_callback,
			1)
		
		self.distSubscription  # prevent unused variable warning
		self.stateSubscription  # prevent unused variable warning
		self.publisher = self.create_publisher(Int32, 'gripperState', 1)
		self.singleFlag = False
	
	def state_callback(self, msg):
		self.currentState = msg.data
		self.get_logger().info('I heard state: "%s"' % msg.data)
		#Modify depending on states
		# State 0 -> searching for thash, gripper closed.
		if self.currentState == 0:
			self.moveGripper(0)
			self.singleFlag = True

		# State 1 -> trash found, open gripper
		elif self.currentState == 1 and singleFlag == True:
			self.moveGripper(1)
			singleFlag = False
		elif self.currentState == 3:
			self.moveGripper(1)

	def distance_callback(self, msg):
		distance = msg.data
		self.get_logger().info('I heard distance: "%s"' % msg.data)

		#Change state and distance to correct values
		# State 1 and trash is close enough and gripper is open -> close gripper
		if self.currentState == 1 and distance < CLOSING_DISTANCE:
			self.moveGripper(0)

	def moveGripper(self, state):
		if state == 0:
			claw_pwm.ChangeDutyCycle(9.5)
		else:
			claw_pwm.ChangeDutyCycle(11.3)

		msg = Int32()
		msg.data = state
		self.publisher.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
	rclpy.init(args=args)

	gripperClaw = Gripper()

	rclpy.spin(gripperClaw)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
