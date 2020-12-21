#!/usr/bin/env python

"""
ROS node for Face Detection

Node publishes: TODO
'camera/faces' --> ImageObjectList

Custom messages:
->ImageObject.msg
    float32 x
    float32 y
    float32 width
    float32 height

->ImageObjectList.msg
    ImageObject[] objects

"""

print("################### Not adapted to ROS 2 yet!! ###################")
import sys
sys.exit("Error message")

import os
import sys

import time

# Command line interface https://click.palletsprojects.com/en/7.x/ 
import click

# ROS
# ROS 2 Imports
import rclpy
from rclpy.node import Node

# ROS types for publishing
from std_msgs.msg import String, Int32, Bool, Float32
from camera.msg import ImageObject, ImageObjectList



# OpenCV for Face Detection
import numpy as np
import cv2

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 30fps 
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen
def gstreamer_pipeline (capture_width=3280, capture_height=2464, display_width=820, display_height=616, framerate=21, flip_method=0) :   
    return ('nvarguscamerasrc ! ' 
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))





# class Face(object):
#   def __init__(self, x, y, w, h):
#     self.x = int(x + w / 2)
#     self.y = int(y + h / 2)
#     self.width = w
#     self.height = h




class FaceDetectNode(Node):
  def __init__(self, show, width, height, displaywidth, displayheight, framerate, flip):
    super().__init__('camera_node')
    
    rclpy.get_default_context().on_shutdown(self.onShutdown)

    ### Attributes for video capture 
    self.show = show
    self.width = width
    self.height = height
    self.displaywidth = displaywidth
    self.displayheight = displayheight
    self.framerate = framerate
    self.flip = flip

    self.face_cascade = cv2.CascadeClassifier('/usr/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml')
    self.eye_cascade = cv2.CascadeClassifier('/usr/share/OpenCV/haarcascades/haarcascade_eye.xml')

    ### Video Capture
    self.cap = cv2.VideoCapture(gstreamer_pipeline(self.width, self.height, self.displaywidth, self.displayheight, self.framerate, self.flip), cv2.CAP_GSTREAMER)

    if (self.show): cv2.namedWindow('Face Detect', cv2.WINDOW_AUTOSIZE)

    if (self.cap.isOpened()): 
      self.get_logger().info("Opened Video Capture pipeline")
    else: 
      self.get_logger().error("Could not open Video Capture pipeline. Check if camera works properly!")
      sys.exit(-1)

    self.lastImage = None
    self.fetchNextImage()

    ### define and announce publisher topics for ROS 
    self.pubFaces = self.create_publisher(ImageObjectList, 'camera/faces', 10)
    # self.pubFaces = self.create_publisher(Image, 'camera/video', 10) # TODO

    ### define and announce subscriber topics
    self.subLed = self.create_subscription(Bool, "camera/activate", self.onActivate, 10)

    # update rate for the reSpeaker values
    self.updateRate = 5

    # Timer for ROS events and callbacks
    self.mainTimer = self.create_timer(1.0 / self.updateRate, self.onTimer)
    self.ledTimer = None

    self.get_logger().info("Started camera and face detection node")

  def getImageObject(self, x, y, w, h):
    o = ImageObject()
    o.x = float( ((x + w / 2.0) / self.width) * 100.0) 
    o.y = float((100.0 * ((y + h / 2)) / self.height))
    o.width = float( (w * 100.0) / self.width )
    o.height = float( (h * 100.0) / self.height )
    #o.width = float((w / self.width) * 100.0)
    #o.height = float((h / self.height) * 100.0)

    print(o)

    return o

  ### Method to detect faces in an image
  def getFaces(self, img):
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

    faceObjects = []
    for (x, y, w, h) in faces:
      faceObjects.append(self.getImageObject(x, y, w, h))
      self.get_logger().info("Found face @ " + str(x) + " " + str(y) + " " + str(w) + " " + str(h) )

    if (self.show):
      for (x,y,w,h) in faces:
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = img[y:y+h, x:x+w]
        eyes = self.eye_cascade.detectMultiScale(roi_gray)
        for (ex,ey,ew,eh) in eyes:
          cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

        cv2.imshow('Face Detect',img)
        keyCode = cv2.waitKey(30) & 0xff
        # Stop the program on the ESC key
        if keyCode == 27:
            raise StopIteration()
    
    return faceObjects


  ### Shutdown method that closes the video capture
  def onShutdown(self):
    try:
      if (self.cap):
        
        self.get_logger().info("Try releasing video capture...")
        self.cap.release()
        self.get_logger().info("Released video capture")
      cv2.destroyAllWindows()
    except:
      pass
    finally:
      self.cap = None

  def fetchNextImage(self):
    # get image from video capture
    ret, self.lastImage = self.cap.read()

  ### Timer function for image processing
  def onTimer(self, event):

    stamp = event.current_real or time.time()

    # get faces from image
    faces = self.getFaces(self.lastImage)

    # Publish faces
    if len(faces) > 0:
      self.get_logger().info("Got " + str(len(faces)) + ": " + str(faces[0].x))
      faceList = ImageObjectList()
      faceList.objects = faces
      self.pubFaces.publish(faceList)


  def onActivate(self, msg):
    ### TODO ###
    pass


@click.command()
@click.option('-s', '--show', is_flag=True,                             help="Show image window")
@click.option('-w', '--width', default=3280, show_default=True,         help="Camera capture width (pixel)  ")
@click.option('-h', '--height', default=2464, show_default=True,        help="Camera capture height (pixel) ")
@click.option('-W', '--displaywidth', default=3280, show_default=True,   help="Image display width (pixel)   ")
@click.option('-H', '--displayheight', default=2464, show_default=True,  help="Image display height (pixel)  ")
@click.option('-r', '--framerate', default=20, show_default=True,       help="Image framerate")
@click.option('-f', '--flip', default=0, show_default=True,             help="Flip Method (see usage)")
def main(show, width, height, displaywidth, displayheight, framerate, flip):
  """
  Start the faceDetect ROS Node. See parameters for configuration
  \b
  Flip Method:
    (0): none             - Identity (no rotation)
    (1): counterclockwise - Rotate counter-clockwise 90 degrees
    (2): rotate-180       - Rotate 180 degrees
    (3): clockwise        - Rotate clockwise 90 degrees
    (4): horizontal-flip  - Flip horizontally
    (5): upper-right-diagonal - Flip across upper right/lower left diagonal
    (6): vertical-flip    - Flip vertically
    (7): upper-left-diagonal - Flip across upper left/low")
  """
  
  # if (show):
  #   print "Show"
  # else: 
  #   print "Not show"

  node = None

  try:
  
    rclpy.init(args=args) # 'camera_node'

    node = FaceDetectNode(show, width, height, displaywidth, displayheight, framerate, flip)

    while not rospy.is_shutdown():
      node.fetchNextImage()

  except (rospy.ROSInterruptException, KeyboardInterrupt):
    node.onShutdown()

if __name__ == "__main__":
  main()
