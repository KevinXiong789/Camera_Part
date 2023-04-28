'''
#**************source code*************************can run
from PyNuitrack import py_nuitrack
import cv2
from itertools import cycle
import numpy as np
from std_msgs.msg import Float32
import time
from influxdb_client import InfluxDBClient, Point

def draw_skeleton(image):
	point_color = (59, 164, 0)
	for skel in data.skeletons:
		for el in skel[1:]:
			x = (round(el.projection[0]), round(el.projection[1]))
			cv2.circle(image, x, 8, point_color, -1)

#get the position (xzd) of right hand, and send these Info with time stamp to InfluxDB
def right_hand_position():
	for skeleton in data.skeletons:
		#turn unit to meter
		right_x=(skeleton.right_hand.real[0])*0.001
		right_y=(-skeleton.right_hand.real[1])*0.001
		#here nuitrack coordinate system (y) is different with D435 camera in pointcloud
		right_d=(skeleton.right_hand.real[2])*0.001
	
		print("right hand x: %.4f, y: %.4f, d: %.4f"%(right_x,right_y,right_d))
		timestamp = int(time.time()*1000)
		data_point = Point("right_hand_position") \
					.field("x", right_x) \
					.field("y", right_y) \
					.field("d", right_d) \
					.time(timestamp,"ms")
		influxdb_write_api.write(bucket="min_distance_test", record=data_point)
		

nuitrack = py_nuitrack.Nuitrack()
nuitrack.init()

devices = nuitrack.get_device_list()
for i, dev in enumerate(devices):
	print(dev.get_name(), dev.get_serial_number())
	if i == 0:
		#dev.activate("ACTIVATION_KEY") #you can activate device using python api
		print(dev.get_activation())
		nuitrack.set_device(dev)


print(nuitrack.get_version())
print(nuitrack.get_license())

nuitrack.create_modules()
nuitrack.run()

#set necessary info to connect InfluxDBClient
influxdb_client = InfluxDBClient(url="http://localhost:8086", \
								token="EZZyYWflA8jgJFT1J5TfTkTbgECQQzcIbEXvTDKwBVKntwRm4JyAEy3wzjzJE20i-i-8k9vFbIO1WDxsGNQSPw==", \
								org="PointCloud")
influxdb_write_api = influxdb_client.write_api()

modes = cycle(["depth", "color"])
mode = next(modes)
while 1:
	key = cv2.waitKey(1)
	nuitrack.update()
	data = nuitrack.get_skeleton()
	data_instance=nuitrack.get_instance()
	img_depth = nuitrack.get_depth_data()
	if img_depth.size:
		cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
		img_depth = np.array(cv2.cvtColor(img_depth,cv2.COLOR_GRAY2RGB), dtype=np.uint8)
		img_color = nuitrack.get_color_data()
		draw_skeleton(img_depth)
		draw_skeleton(img_color)
		right_hand_position()
		#right_hand_position(img_color)
		#draw_face(img_depth)
		#draw_face(img_color)
		if key == 32:
			mode = next(modes)
		if mode == "depth":
			cv2.imshow('Image', img_depth)
		if mode == "color":
			if img_color.size:
				cv2.imshow('Image', img_color)
	if key == 27:
		break

nuitrack.release()
'''
'''
#****************minimize source code*************can run
from PyNuitrack import py_nuitrack
import cv2
import numpy as np
import time
from influxdb_client import InfluxDBClient, Point

def draw_skeleton(image):
	point_color = (59, 164, 0)
	for skel in data.skeletons:
		for el in skel[1:]:
			x = (round(el.projection[0]), round(el.projection[1]))
			cv2.circle(image, x, 8, point_color, -1)

#get the position (xzd) of right hand, and send these Info with time stamp to InfluxDB
def right_hand_position():
	for skeleton in data.skeletons:
		#turn unit to meter
		right_x=(skeleton.right_hand.real[0])*0.001
		right_y=(-skeleton.right_hand.real[1])*0.001
		#here nuitrack coordinate system (y) is different with D435 camera in pointcloud
		right_d=(skeleton.right_hand.real[2])*0.001
	
		print("right hand x: %.4f, y: %.4f, d: %.4f"%(right_x,right_y,right_d))
		timestamp = int(time.time()*1000)
		data_point = Point("right_hand_position") \
					.field("x", right_x) \
					.field("y", right_y) \
					.field("d", right_d) \
					.time(timestamp,"ms")
		influxdb_write_api.write(bucket="min_distance_test", record=data_point)
		

nuitrack = py_nuitrack.Nuitrack()
nuitrack.init()

nuitrack.create_modules()
nuitrack.run()

#set necessary info to connect InfluxDBClient
influxdb_client = InfluxDBClient(url="http://localhost:8086", \
								token="EZZyYWflA8jgJFT1J5TfTkTbgECQQzcIbEXvTDKwBVKntwRm4JyAEy3wzjzJE20i-i-8k9vFbIO1WDxsGNQSPw==", \
								org="PointCloud")
influxdb_write_api = influxdb_client.write_api()

while 1:
	key = cv2.waitKey(1)
	nuitrack.update()
	data = nuitrack.get_skeleton()
	data_instance=nuitrack.get_instance()
	img_depth = nuitrack.get_depth_data()
	
	cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
	img_depth = np.array(cv2.cvtColor(img_depth,cv2.COLOR_GRAY2RGB), dtype=np.uint8)
	
	draw_skeleton(img_depth)
	#draw_skeleton(img_color)
	
	right_hand_position()
	cv2.imshow('Image', img_depth)
		
	if key == 27:
		break

nuitrack.release()
'''


'''
#*****************Nuitrack+InfluxDB+Publisher*******************can run
from PyNuitrack import py_nuitrack
import cv2
import numpy as np
import time
from influxdb_client import InfluxDBClient, Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class NuitrackPublisher(Node):
	def __init__(self):
		super().__init__('nuitrack_publisher')
		self.publisher = self.create_publisher(Float32MultiArray, '/Nuitrack/right_hand_position', 1)
		#set necessary info to connect InfluxDBClient
		self.influxdb_client = InfluxDBClient(url="http://localhost:8086", \
										token="EZZyYWflA8jgJFT1J5TfTkTbgECQQzcIbEXvTDKwBVKntwRm4JyAEy3wzjzJE20i-i-8k9vFbIO1WDxsGNQSPw==", \
										org="PointCloud")
		self.influxdb_write_api = self.influxdb_client.write_api()
		

	def draw_skeleton(self, image):
		point_color = (59, 164, 0)
		for skel in self.data.skeletons:
			for el in skel[1:]:
				x = (round(el.projection[0]), round(el.projection[1]))
				cv2.circle(image, x, 8, point_color, -1)

	def right_hand_position(self):
		
		for skeleton in self.data.skeletons:
			# turn unit to meter
			right_x = (skeleton.right_hand.real[0]) * 0.001
			right_y = (-skeleton.right_hand.real[1]) * 0.001
			# here nuitrack coordinate system (y) is different with D435 camera in pointcloud
			right_d = (skeleton.right_hand.real[2]) * 0.001

			print("right hand x: %.4f, y: %.4f, d: %.4f" % (right_x, right_y, right_d))

			timestamp = int(time.time()*1000)
			data_point = Point("right_hand_position") \
						.field("x", right_x) \
						.field("y", right_y) \
						.field("d", right_d) \
						.time(timestamp,"ms")
			self.influxdb_write_api.write(bucket="min_distance_test", record=data_point)

			msg = Float32MultiArray()
			msg.data = [right_x, right_y, right_d]
			self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	nuitrack_publisher = NuitrackPublisher()

	nuitrack = py_nuitrack.Nuitrack()
	nuitrack.init()
	nuitrack.create_modules()
	nuitrack.run()

	while True:
	#while rclpy.ok():
		nuitrack.update()
		nuitrack_publisher.data = nuitrack.get_skeleton()
		img_depth = nuitrack.get_depth_data()

		cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
		img_depth = np.array(cv2.cvtColor(img_depth, cv2.COLOR_GRAY2RGB), dtype=np.uint8)

		nuitrack_publisher.draw_skeleton(img_depth)
		nuitrack_publisher.right_hand_position()
		cv2.imshow('Image', img_depth)

		if cv2.waitKey(1) & 0xFF == 27:
			break

	nuitrack.release()
	#nuitrack_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
'''
	  
'''
#******************min code of nuitrack(just for test)***********************can run
from PyNuitrack import py_nuitrack
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class NuitrackPublisher(Node):
	def __init__(self):
		super().__init__('nuitrack_publisher')
		self.publisher = self.create_publisher(Float32MultiArray, '/Nuitrack/right_hand_position', 1)
	def draw_skeleton(self, image):
		point_color = (59, 164, 0)
		for skel in self.data.skeletons:
			for el in skel[1:]:
				x = (round(el.projection[0]), round(el.projection[1]))
				cv2.circle(image, x, 8, point_color, -1)
	def right_hand_position(self):
		for skeleton in self.data.skeletons:
			# turn unit to meter
			right_x = (skeleton.right_hand.real[0]) * 0.001
			right_y = (-skeleton.right_hand.real[1]) * 0.001
			# here nuitrack coordinate system (y) is different with D435 camera in pointcloud
			right_d = (skeleton.right_hand.real[2]) * 0.001
			print("right hand x: %.4f, y: %.4f, d: %.4f" % (right_x, right_y, right_d))
			msg = Float32MultiArray()
			msg.data = [right_x, right_y, right_d]
			self.publisher.publish(msg)
def main(args=None):
	rclpy.init(args=args)
	nuitrack_publisher = NuitrackPublisher()
	#rclpy.spin(nuitrack_publisher)
	nuitrack = py_nuitrack.Nuitrack()
	nuitrack.init()
	nuitrack.create_modules()
	nuitrack.run()
	while True:
	#while rclpy.ok():
		nuitrack.update()
		nuitrack_publisher.data = nuitrack.get_skeleton()
		img_depth = nuitrack.get_depth_data()
		cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
		img_depth = np.array(cv2.cvtColor(img_depth, cv2.COLOR_GRAY2RGB), dtype=np.uint8)
		nuitrack_publisher.draw_skeleton(img_depth)
		nuitrack_publisher.right_hand_position()
		cv2.imshow('Image', img_depth)
		if cv2.waitKey(1) & 0xFF == 27:
			break
	nuitrack.release()
	#nuitrack_publisher.destroy_node()
	rclpy.shutdown()
if __name__ == '__main__':
	main()
'''



#********************Nuitrack+InfluxDB+Publisher another form************************************can run
#****************Use Nuitrack get right hand and elbow position, publish them as topic and send hand Position to InfluxDB**************
from PyNuitrack import py_nuitrack
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from influxdb_client import InfluxDBClient, Point

class NuitrackPublisher(Node):
	def __init__(self):
		super().__init__('nuitrack_publisher')
		self.publisher = self.create_publisher(Float32MultiArray, '/Nuitrack/right_hand_position', 1)
		self.nuitrack = py_nuitrack.Nuitrack()
		self.nuitrack.init()
		
		devices = self.nuitrack.get_device_list()
		i = 1
		dev = devices[i] # Set i to the index of the device you want to select
		print(dev.get_name(), dev.get_serial_number())
		self.nuitrack.set_device(dev)
		
		self.nuitrack.create_modules()
		self.nuitrack.run()
		#set necessary info to connect InfluxDBClient
		self.influxdb_client = InfluxDBClient(url="http://localhost:8086", \
										token="EZZyYWflA8jgJFT1J5TfTkTbgECQQzcIbEXvTDKwBVKntwRm4JyAEy3wzjzJE20i-i-8k9vFbIO1WDxsGNQSPw==", \
										org="PointCloud")
		self.influxdb_write_api = self.influxdb_client.write_api()

	def draw_skeleton(self, image):
		point_color = (59, 164, 0)
		for skel in self.data.skeletons:
			for el in skel[1:]:
				x = (round(el.projection[0]), round(el.projection[1]))
				cv2.circle(image, x, 8, point_color, -1)

	def right_hand_position(self):
		for skeleton in self.data.skeletons:
			# turn unit to meter
			right_x = (skeleton.right_hand.real[0]) * 0.001
			right_y = (-skeleton.right_hand.real[1]) * 0.001
			# here nuitrack coordinate system (y) is different with D435 camera in pointcloud
			right_d = (skeleton.right_hand.real[2]) * 0.001
			print("right hand x: %.4f, y: %.4f, d: %.4f" % (right_x, right_y, right_d))

			# turn unit to meter
			right_elbow_x = (skeleton.right_elbow.real[0]) * 0.001
			right_elbow_y = (-skeleton.right_elbow.real[1]) * 0.001
			# here nuitrack coordinate system (y) is different with D435 camera in pointcloud
			right_elbow_d = (skeleton.right_elbow.real[2]) * 0.001
			#print("right elbow x: %.4f, y: %.4f, d: %.4f" % (right_elbow_x, right_elbow_y, right_elbow_d))

			
			# Right hand Daten to InfluxDB send
			timestamp = int(time.time()*1000)
			data_point = Point("right_hand_position") \
						.field("x", right_x) \
						.field("y", right_y) \
						.field("d", right_d) \
						.time(timestamp,"ms")
			self.influxdb_write_api.write(bucket="min_distance_test", record=data_point)

			# publish right hand and right elbow position
			msg = Float32MultiArray()
			msg.data = [right_x, right_y, right_d, right_elbow_x, right_elbow_y, right_elbow_d]
			self.publisher.publish(msg)

	def run_nuitrack(self):
		while rclpy.ok():
			self.nuitrack.update()
			self.data = self.nuitrack.get_skeleton()
			img_depth = self.nuitrack.get_depth_data()
			cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
			img_depth = np.array(cv2.cvtColor(img_depth, cv2.COLOR_GRAY2RGB), dtype=np.uint8)
			self.draw_skeleton(img_depth)
			self.right_hand_position()
			cv2.imshow('Image', img_depth)
			if cv2.waitKey(1) & 0xFF == 27:
				break
		self.nuitrack.release()

def main(args=None):
	rclpy.init(args=args)
	nuitrack_publisher = NuitrackPublisher()
	nuitrack_publisher.run_nuitrack()
	rclpy.shutdown()

if __name__ == '__main__':
	main()






'''
#*******************Nuitrack get depth data from realsense2_camera******************************testing
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyNuitrack import py_nuitrack

class NuitrackPublisher(Node):
	def __init__(self):
		super().__init__('nuitrack_publisher')
		self.publisher = self.create_publisher(Float32MultiArray, '/Nuitrack/right_hand_position', 1)
		self.bridge = CvBridge()
		self.subscription = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.image_callback, 1)
		#self.subscription  # prevent unused variable warning
		#self.data = None

	def image_callback(self, msg):
		try:
			# Convert the ROS Image message to OpenCV image
			img_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
			cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
			img_depth = np.array(cv2.cvtColor(img_depth, cv2.COLOR_GRAY2RGB), dtype=np.uint8)
			self.draw_skeleton(img_depth)
			cv2.imshow('Image', img_depth)
			cv2.waitKey(1)
			
		except Exception as e:
			self.get_logger().error('Error processing depth image: {}'.format(e))

	def draw_skeleton(self, image):
		point_color = (59, 164, 0)
		for skel in self.data.skeletons:
			for el in skel[1:]:
				x = (round(el.projection[0]), round(el.projection[1]))
				cv2.circle(image, x, 8, point_color, -1)

	def right_hand_position(self):
		for skeleton in self.data.skeletons:
			# turn unit to meter
			right_x = (skeleton.right_hand.real[0]) * 0.001
			right_y = (-skeleton.right_hand.real[1]) * 0.001
			# here nuitrack coordinate system (y) is different with D435 camera in pointcloud
			right_d = (skeleton.right_hand.real[2]) * 0.001
			print("right hand x: %.4f, y: %.4f, d: %.4f" % (right_x, right_y, right_d))
			msg = Float32MultiArray()
			msg.data = [right_x, right_y, right_d]
			self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	nuitrack_publisher = NuitrackPublisher()
	nuitrack = py_nuitrack.Nuitrack()
	nuitrack.init()
	nuitrack.create_modules()
	#nuitrack.run()
	while rclpy.ok():
		#nuitrack.update()
		nuitrack_publisher.data = nuitrack.get_skeleton()
		nuitrack_publisher.right_hand_position()
		rclpy.spin_once(nuitrack_publisher)

	nuitrack.release()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
'''

