#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from object_msgs.msg import Object, ObjectArray
from cv_bridge import CvBridge, CvBridgeError
import cv2

import torch
from darknet import Darknet, load_model
from util import *

class YOLOv3_node():
    def __init__(self):

        print("Initializing...")

        # Set network setting
        rospy.init_node("yolov3_node")
        self.model_path = rospy.get_param("~model_path", "")
        self.weight_path = rospy.get_param("~weight_path", "")
        self.class_path = rospy.get_param("~class_path", "")
        self.color_path = rospy.get_param("~color_path", "")

        self.batch_size = 1
        self.confidence = 0.6
        self.nms_thesh = 0.4

        # self.CUDA = False
        self.CUDA = True if torch.cuda.is_available() else False

        # Set up the neural network
        print("Loading network.....")
        self.model = load_model(self.model_path, self.weight_path, "cuda" if self.CUDA else "cpu")

        print("Network successfully loaded")

        self.model.hyperparams["height"] = 416
        self.inp_dim = int(self.model.hyperparams["height"])

        assert self.inp_dim % 32 == 0
        assert self.inp_dim > 32

        # Set the model in evaluation mode
        self.model.eval()

        self.classes = load_classes(self.class_path)
        self.num_classes = len(self.classes)
        self.colors = load_colors(self.color_path)

        # cv2.namedWindow('YOLO detects', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('jpg', width, height)

        # Set node information
        rospy.Subscriber("image", CompressedImage, self.cb_image, buff_size=10000000, queue_size=1)
        self.pub_detect = rospy.Publisher("/yolov3_detect/compressed", CompressedImage, queue_size=1)
        self.pub_data = rospy.Publisher("/yolov3_detect/objects", ObjectArray, queue_size=1)
        self.objectList = []
        self.bridge = CvBridge()

        print("Network initializing complete")

    def write(self, x, results):
        msg = Object()
        
        c1 = tuple(x[1:3].int())
        c2 = tuple(x[3:5].int())
        img = results
        cls = int(x[-1])
        color = self.colors[cls]
        label = "{0}".format(self.classes[cls])
        msg.xmin_ymin_xmax_ymax = [c1[0].item(), c1[1].item(), c2[0].item(), c2[1].item()]
        msg.class_name = self.classes[cls]

        cv2.rectangle(img, c1, c2, color, 1)

        t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
        c2 = c1[0] + t_size[0] + 3, c1[1] + t_size[1] + 4
        cv2.rectangle(img, c1, c2, color, -1)        
        cv2.putText(img, label, (c1[0], c1[1] + t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 1, [225, 255, 255], 1)

        self.objectList.append(msg)

        return img
    
    def cb_image(self, img_msg):
        try:
            # Convert Image to OpenCV Format (Compressed Image Message to CV2)
            image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
            img = prep_image(image, self.inp_dim)

            if self.CUDA:
                im_dim = image.shape[1], image.shape[0]
                im_dim = torch.cuda.FloatTensor(im_dim).repeat(1, 2)

                img = img.cuda()    

            else:
                im_dim = image.shape[1], image.shape[0]
                im_dim = torch.FloatTensor(im_dim).repeat(1, 2)    

            with torch.no_grad():
                output = self.model(img)
            output = write_results(output, self.confidence, self.num_classes, nms_conf=self.nms_thesh)

            if type(output) != int:
                output[:, 1:5] = torch.clamp(output[:, 1:5], 0.0, float(self.inp_dim))

                im_dim = im_dim.repeat(output.size(0), 1) / self.inp_dim
                output[:, 1:5] *= im_dim 

                list(map(lambda x: self.write(x, image), output))

            imgmsg = CompressedImage()
            imgmsg.header.stamp = rospy.Time.now()
            imgmsg.format = "jpeg"
            imgmsg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
            
            datamsg = ObjectArray()
            datamsg.Objects = self.objectList

            self.pub_detect.publish(imgmsg)
            self.pub_data.publish(datamsg)
            self.objectList = []

            # print("published")

        except CvBridgeError as e:
            print(e)

if __name__=='__main__':
    yolov3_node = YOLOv3_node()
    rospy.sleep(5)
    rospy.spin()
