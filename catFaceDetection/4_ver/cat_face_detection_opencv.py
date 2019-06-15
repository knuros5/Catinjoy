#!/usr/bin/env python2

# coding: utf-8

# ### Reference https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_objdetect/py_face_detection/py_face_detection.html#face-detection
# 
# 
# #### prerequisite :  pip install opencv-python

# In[2]:

import rospy
import numpy as np
import cv2 
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
from time import sleep
from std_msgs.msg import String

#rospy.init_node('cat_face_detection')
#pub = rospy.Publisher('detect',String,queue_size=1)
detectCount = 0

def cat_detector():
	global detectCount
	pub = rospy.Publisher('detect', String, queue_size=10)
	rospy.init_node('cat_face_detector', anonymous = True)
	rate = rospy.Rate(10)
	msg = "cat detected "+ str(detectCount) + "!!!"
	pub.publish(msg)
	print("-----publish msg : cat detected!!!\n")
	detectCount = detectCount + 1
	rate.sleep()


# ### Download haarcascade xml files from https://github.com/opencv/opencv/tree/master/data/haarcascades
# 
# READ LICENSE TERMS BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
# 
# #### curl -O https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalcatface.xml
# 
# 
# #### curl -O https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalcatface_extended.xml

# In[3]:


cat_cascade = cv2.CascadeClassifier('/home/sunny/catkin_ws/src/cat_face_detection/src/haarcascade_frontalcatface.xml')
cat_ext_cascade = cv2.CascadeClassifier('/home/sunny/catkin_ws/src/cat_face_detection/src/haarcascade_frontalcatface_extended.xml')


# ### Set Tunable parameters

# In[4]:

SF=1.05  # try different values of scale factor like 1.05, 1.3, etc
N=3 # try different values of minimum neighbours like 3,4,5,6

isCat=False
IMAGE_DIR = '/home/sunny/Desktop/image/'
TEXTFILENAME = "/home/sunny/catkin_ws/src/cat_face_detection/src/detected_index.txt"
DETECTED_IMAGE_DIR = '/home/sunny/Desktop/detect_image/'



# #### For each image 
# * Read the image
# * convert to gray scale
# * use the above two cascades to get coordinates of rectangles
# * plot the rectangles


# make txt file
f = open(TEXTFILENAME, 'w')


# In[5]:

def processImage(image_dir,image_filename):
	# read the image
	img = cv2.imread(image_dir+image_filename)
	#print("cv2.imread FINISH")
	# convery to gray scale
	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#print("cv2.cvtColor FINISH")
	# this function returns tuple rectangle starting coordinates x,y, width, height
	cats = cat_cascade.detectMultiScale(gray, scaleFactor=SF, minNeighbors=N)
	print("detectMultiScale FINISH")
	#print(cats) # one sample value is [[268 147 234 234]]
	cats_ext = cat_ext_cascade.detectMultiScale(gray, scaleFactor=SF, minNeighbors=N)
	#print("detectMultiScale2 FINISH")
	#print(cats_ext)

	global isCat

	# draw a blue rectangle on the image
	for (x,y,w,h) in cats:
		img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
		isCat = True
		#print("blue rectangle FINISH")
	# draw a green rectangle on the image 
	for (x,y,w,h) in cats_ext:
		img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
		isCat = True
		#print("green rectangle FINISH")
	
	# when cat detected!!!
	count = 0;
	if isCat:
		detect_image_file_name = 'detect_'+image_filename

		# mark cat image index in textfile
		f.write(detect_image_file_name+'\n')
		print('********** CAT DETECTED!!! **********\n')

		# publish detection message
		cat_detector()
		#msg = "cat detected!!"
		#pub.publish(msg)

		# save the image to a file		
		cv2.imwrite(DETECTED_IMAGE_DIR+detect_image_file_name,img)
	
		# show in window
		#img = mpimg.imread(DETECTED_IMAGE_DIR+detect_image_file_name)
		#plt.imshow(img)
		#plt.show()

	
	nKey = cv2.waitKey(30)%255
	if nKey == 27:
		exit()
	#print("isCat if phrase FINISH")
	isCat = False



# #### There are 6 images in cats directory on which we have to run cat face detection

# In[6]:

idx = 100
#rate = rospy.Rate(10)
#while not rospy.is_shutdown():
while idx < 9990:
	# set file name
	if idx <= 999:
		filename = 'left0'+str(idx)+'.jpg'
	else:
		filename = 'left'+str(idx)+'.jpg'
	print(IMAGE_DIR+filename)
	
	# open image file
	try:
		f1 = open(IMAGE_DIR+filename,'r')
	except IOError as e:
		print(e)
		idx = idx - 5

	# process image file
	try:
		processImage(IMAGE_DIR,filename)
	except Exception as e:
		print(e)
		sleep(2)

	idx = idx + 5
	if idx > 900:
		break
	

	# end of while


f.close()

