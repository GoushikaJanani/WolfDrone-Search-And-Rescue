#!/usr/bin/python
# python-v4l2capture
#
# This file is an example on how to capture a mjpeg video with
# python-v4l2capture.
#
# 2019 Janani, Bharat
#


from PIL import Image
import select
import v4l2capture
import time
import io
import cv2
import numpy as np


class Camera():
	def __init__(self):
		self.video = v4l2capture.Video_device("/dev/video0")
		size_x,size_y = self.video.set_format(1920, 1080,fourcc='MJPG')
		self.video.create_buffers(1)
		self.video.queue_all_buffers()
		self.video.start()
		self.frame_list = list()

	def read_frame(self):
	
		select.select((self.video,), (), ())
		image_data = self.video.read_and_queue()

	def read_frame_split(self):
		self.frame_list = list()
		select.select((self.video,), (), ())
		image_data = self.video.read_and_queue()
		print(len(image_data))
		
		frame = cv2.imdecode(np.frombuffer(image_data,dtype=np.uint8),1)
		print("Frame shape")
		print(frame.shape)
		x,y,z = frame.shape
		xby2 = int(x/2)
		yby2 = int(y/2)
		xby4 = int(x/4)
		yby4 = int(y/4)
		# Split one image into 5 images for yolo detection
		self.frame_list.append(frame[xby4:xby4+xby2,yby4:yby4+yby2,:])
		self.frame_list.append(frame[:xby2,:yby2,:])
		self.frame_list.append(frame[xby2:,:yby2,:])
		self.frame_list.append(frame[:xby2,yby2:,:])
		self.frame_list.append(frame[xby2:,yby2:,:])
		print("read frames")
		return self.frame_list,frame

