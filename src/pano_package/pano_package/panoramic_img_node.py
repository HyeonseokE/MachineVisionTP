import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np      

class ImageTransformToPANORAMIC(Node):
  def __init__(self):
    super().__init__('image_transform_to_panoramic')
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
    self.pub_image_raw1 = self.create_publisher(Image, '/camera1/sub_image_raw', 10)
    self.pub_image_raw2 = self.create_publisher(Image, '/camera2/sub_image_raw', 10)
    self.pub_PANORAMIC_image = self.create_publisher(Image, 'panoramic_image', 10)
    self.image = np.empty(shape=[1]) 
    ## camera2 image buffer
    self.buffer1 = []
    self.buffer2 = []
    ## stitcher
    self.stitcher = cv2.Stitcher_create()
    
  # <algorithm>
  # 1. first, stack camera2 image to buffer about 3frames
  # 2. then, camera1 image input started
  # 3. when camera1 input interrupt occur, pop the last frame of camera2
  # 4. then, start image stitching and iterate 2 ~ 4
  
  def image_buffer(self, buffer, data):
    if len(buffer) >= 5:
      del buffer[0]
    buffer.append(data)
    print(len(buffer))
    
    return buffer
  
  def get_PANORNAMA(self, data, frame_id):
    # stack data to buffer
    if frame_id == 'camera1':
      self.buffer1 = self.image_buffer(self.buffer1, data)
    else:
      self.buffer2 = self.image_buffer(self.buffer2, data)
    # implement panorama
    if len(self.buffer1) >= 1 and len(self.buffer2) >= 1:
      print(self.buffer1[-1].shape,self.buffer2[-1].shape)
      ret, pano_image = self.stitcher.stitch([self.buffer2[-1],self.buffer1[-1]])
      if ret != cv2.Stitcher_OK:
        print('stitcher failed')
        if ret == 1:
          print(f'{ret}, less matching points')
        elif ret == 2:
          print(f'{ret}, fail to transform')
        else:
          print(f'{ret}, camera position error')
        return 0
      cv2.namedWindow('panoramic_image', cv2.WINDOW_NORMAL)
      cv2.imshow('panoramic_image', pano_image)
      cv2.waitKey(33)
      img_msg = self.bridge.cv2_to_imgmsg(pano_image) # image type transform(numpy->sensor.msgs.Image)(search documentation)
      self.pub_PANORAMIC_image.publish(img_msg) # publish image

  
  def image_callback_1(self, data): # allocate what subsciber got to valable(data)
    frame_id = data.header.frame_id
    self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8') # image type transform(sensor.msgs.Image->numpy)
    self.get_PANORNAMA(self.image, frame_id)
    cv2.imshow('input1/image_raw1', self.image) # visualization to confirm well working
    cv2.waitKey(33)
    # img_msg = self.bridge.cv2_to_imgmsg(self.image) # image type transform(numpy->sensor.msgs.Image)(search documentation)
    # self.pub_bev_image1.publish(img_msg) # publish image


  def image_callback_2(self, data):
    frame_id = data.header.frame_id
    self.image = self.bridge.imgmsg_to_cv2(data, 'bgr8') # image type transform(sensor.msgs.Image->numpy)
    self.get_PANORNAMA(self.image, frame_id)
    cv2.imshow('input2/image_raw2', self.image) # visualization to confirm well working
    cv2.waitKey(33)
    # img_msg = self.bridge.cv2_to_imgmsg(self.image) # image type transform(numpy->sensor.msgs.Image)(search documentation)
    # self.pub_bev_image2.publish(img_msg) # publish image

def main():
  rclpy.init() # rclpy init
  node = ImageTransformToPANORAMIC() # create Node

  try:
    rclpy.spin(node) # start node iteration
  except KeyboardInterrupt:
    node.get_logger().info('[camera1]:stopped by keyboard\n[camera2]:stopped by keyboard') # Get the nodes logger
  finally:
    node.destroy_node() # Frees resources used by the node, including any entities created by the following
    rclpy.shutdown() # del rclpy

if __name__ == '__main__':
  main()