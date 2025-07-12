#
/usr/bin/env python3
# -*- coding: UTF-8 -*-
from PIL import Image
import pytesseract
import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
from std_msgs.msg import String

class ReadImage():
    def __init__(self):         
        # 开启d435的数据流
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)        
        # Start streaming
        self.pipeline.start(self.config)
    
    def read(self): 
        #等待一对连贯的颜色帧
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())        
        x, y = color_image.shape[0:2]
        # 缩小图像的大小
        color_image = cv2.resize(color_image, (int(y / 2), int(x / 2)))
        #cv2.imwrite('test.png',color_image)  # 写入图片
        # 转灰度图
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # 二值化图片
        retVal,binary = cv2.threshold(gray, 100 ,100, cv2.THRESH_BINARY)           
        # 返回图像                            
        return binary,color_image



def detect_word(target_image):
    text = pytesseract.image_to_string(target_image)
ZZ    return text  


def start():
    # 定义发布者
    pub = rospy.Publisher('/detect_word_reslut',String, queue_size=10)
    # 初始化节点
    rospy.init_node('detect_node',anonymous=True)
    # 设置发布消息频率
    rate = rospy.Rate(50)
    # 实例化ReadImage（）
    read_img = ReadImage()
      
    while not rospy.is_shutdown():
        # 获取图像
        image_binary,image_color = read_img.read()
        # 判断图像是否为空
        if image_binary is not None: 
            # 在窗口显示图片         
            cv2.imshow('Realsense', image_color)
            # 识别图片中的文字
            text = detect_word(image_binary)
            # 过滤掉识别数来的乱码，只保留数字和字母
            s="".join(filter(str.isalnum,text))
            if not s.strip()==' ':
                # 发布识别出来的文字
                pub.publish(s)
                
            
        else:
            print("can not read image") 
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
            #cv2.waitKey(0)
        
   
        rate.sleep()


if __name__=='__main__':
    start()

    
