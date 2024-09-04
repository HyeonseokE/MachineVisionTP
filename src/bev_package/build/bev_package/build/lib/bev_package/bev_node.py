import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np      

IMAGE_W = 640
IMAGE_H = 480
DISTORTION = 270
#BEV_H = 250
BEV_H = 480
BEV_W = 640

class ImageTransformToBEV(Node):
  def __init__(self):
    super().__init__('imageraw_subscriber')
    self.bridge = CvBridge() # bridge instance alloc
    qos = QoSProfile(depth=10) # quality of service

    ## subscriber initalization
    self.sub_camera1 = self.create_subscription(
    Image, # message type you want to subscribe 
    '/camera1/image_raw', # topic name
    self.image_callback_1, # user defined callback function
    qos)
    self.sub_camera2 = self.create_subscription(Image, '/camera2/image_raw', self.image_callback_2, qos)
    ## publisher initalization
    self.pub_bev_image1 = self.create_publisher(Image, '/camera1/bev_image', 10)
    self.pub_bev_image2 = self.create_publisher(Image, '/camera2/bev_image', 10)

    self.image = np.empty(shape=[1])

  def get_Homography(self, **kwargs): # old one = 'old', new one = 'new' for parameter setting
    isOld = kwargs['isOld']
    global BEV_H, BEV_W
    # 12
    # 34
    if isOld == 'old': # it's old camera, so allocating old_camera's parameter
      points = np.float32([[206,280], [437, 280], [20,480], [627, 480]])
    else: # it's new camera, so allocating new_camera's parameters
      points = np.float32([[235,125], [421,125],[10, 463], [639, 463]])
      
    dest = np.float32([[0, 0], [BEV_W, 0], [0, BEV_H], [BEV_W, BEV_H]])

    M = cv2.getPerspectiveTransform(points, dest) # The transformation matrix
    invM = cv2.getPerspectiveTransform(dest, points) # Inverse transformation
    print("MATRIX  1{}".format(M))
    print("MATRIX  2{}".format(invM))
    return M, invM

  def get_BEV(self, img, **kwargs):
    isOld = kwargs['camera']
    M, invM = self.get_Homography(isOld = isOld)
    BEV = cv2.warpPerspective(img, M, (BEV_W, BEV_H)) # get BEV
    return BEV
  
  def image_callback_1(self, data): # allocate what subsciber got to valable(data)
    self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8') # image type transform(sensor.msgs.Image->numpy)
    bev_img = self.get_BEV(self.image, camera='old')
    cv2.imshow('camera1_bev', bev_img) # visualization to confirm well working
    cv2.waitKey(33)
    img_msg = self.bridge.cv2_to_imgmsg(bev_img) # image type transform(numpy->sensor.msgs.Image)(search documentation)
    self.pub_bev_image1.publish(img_msg) # publish image


  def image_callback_2(self, data):
    self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8') # image type transform(sensor.msgs.Image->numpy)
    bev_img = self.get_BEV(self.image, camera='new')
    cv2.imshow('camera2_bev', bev_img) # visualization to confirm well working
    cv2.waitKey(33)
    img_msg = self.bridge.cv2_to_imgmsg(bev_img) # image type transform(numpy->sensor.msgs.Image)(search documentation)
    self.pub_bev_image2.publish(img_msg) # publish image

def main():
  rclpy.init() # rclpy init
  node = ImageTransformToBEV() # create Node

  try:
    rclpy.spin(node) # start node iteration
  except KeyboardInterrupt:
    node.get_logger().info('[camera1]:stopped by keyboard\n[camera2]:stopped by keyboard') # Get the nodes logger
  finally:
    node.destroy_node() # Frees resources used by the node, including any entities created by the following
    rclpy.shutdown() # del rclpy

if __name__ == '__main__':
  main()