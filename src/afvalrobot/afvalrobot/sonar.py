import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float64

TRIGGER = 10
ECHO = 11

GPIO.setmode(GPIO.BCM) # BCM is GPIO numbers
GPIO.setup(TRIGGER, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

class SonarPublisher(Node):

	def __init__(self):
		super().__init__('sonar_publisher')
		self.publisher_ = self.create_publisher(Float64, 'distance', 1)
		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		
		
	def timer_callback(self):
		#timeStart = time.time()
		GPIO.output(TRIGGER, GPIO.HIGH)
		time.sleep(0.000010)
		GPIO.output(TRIGGER, GPIO.LOW)
		
		while(not GPIO.input(ECHO)):
			timeStart = time.time_ns()

		while(GPIO.input(ECHO)):
			timeEcho = time.time_ns()   

		distance = (float)(timeEcho - timeStart)/(58*1000)

		#test if timeout is needed
		#while(not GPIO.input(ECHO)):
		#    pass
		#timeEcho = time.time()
		#distance = ((timeEcho - timeStart) * 340)/2
		
		msg = Float64() 
		msg.data = distance
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
	rclpy.init(args=args)

	sonar_publisher = SonarPublisher()

	rclpy.spin(sonar_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
