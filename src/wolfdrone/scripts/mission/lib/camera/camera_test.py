import cv2
import numpy as np
import yolo
import v4l2_cam as cam
import time

# Init neural network
print("Initializing YoloV3")
net,meta = yolo.init_yolo()

# Init camera
print("Creating Video Capture Object")
video = cam.set_camera()

#Start video 
video.start()
j = 0

bbox  =list()

#correction
correction = [(480,270),(0,0),(0,540),(960,0),(960,540)]

try:
	while j < 1:
		print(j)
		frame = cam.read_frame(video)
		frames,image = cam.read_frame_split(video)

		print("Frames read")
		
		start_time = time.time()
		found = 0
		for i,frame in enumerate(frames):
			#Correction offsets for split frames
			c_x,c_y = correction[i]
			r = yolo.detect_np(net,meta,frame)

			if r== None:
				continue
			for (name,prob,(x1,y1,x2,y2)) in r:
				if name == b'person':
					bbox.append((x1+c_x,y1+c_y,x2,y2))
					found = 1
					break
			if found:
				break
				
		(x1,y1,x2,y2) = bbox[0]
		end_time = time.time()
		print(end_time - start_time)

		cv2.putText(image,"person" ,(int(x1-x2/2) -2,int(y1-y2/2)-2),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
		cv2.rectangle(image,(int(x1-x2/2),int(y1-y2/2)),(int(x1+x2/2),int(y1+y2/2)),(0,0,1),2)
		
		
		cv2.namedWindow('final',cv2.WINDOW_NORMAL)
		cv2.imshow('final',image)
		key = cv2.waitKey(0)
		j = j+1
	video.close()
	
except Exception as e:
	print(e)
	video.close()
