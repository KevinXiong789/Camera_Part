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

