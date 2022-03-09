import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import rospkg
import os
import loadcpt

__author__ = "Alexander Carballo"
__email__ = "alexander@g.sp.m.is.nagoya-u.ac.jp"

"""
   Thermal2Color
   Converts grayscale thermal camera image topics to color using a palette 
"""
def package(path = ""):
    rospack = rospkg.RosPack()
    return os.path.join(rospack.get_path("thermal2color"), path)

class Thermal2Color(object):
    def __init__(self):
        self.ros_init()

    def ros_init(self):
        rospy.init_node('thermal2color', anonymous=True)
        self.img_topic = rospy.get_param("~image", "/image_raw")
        palettename = rospy.get_param("~palette", "cequal")
        self.xmin = rospy.get_param("~xmin", np.NaN)
        self.ymin = rospy.get_param("~ymin", np.NaN)
        self.xmax = rospy.get_param("~xmax", np.NaN)
        self.ymax = rospy.get_param("~ymax", np.NaN)
        rospy.loginfo("input image topic: \"%s\", palette: %s", self.img_topic, palettename)
        self.colormap = loadcpt.gmtColormap(package('palettes/'+palettename+'.cpt'), palettename)
        self.pub_color = rospy.Publisher(self.img_topic + "_gray2color", Image, queue_size=10)
        rospy.Subscriber(self.img_topic, Image, self.img_callback)
        self.bridge = CvBridge()

    def apply_custom_colormap(self, image, cmapname='inferno'):
        assert image.dtype == np.uint8, 'must be np.uint8 image'
        if image.ndim == 3: 
            image = image.squeeze(-1)

        # Initialize the matplotlib color map
        #cmap = plt.get_cmap(cmapname)
        cmap=self.colormap
        sm = plt.cm.ScalarMappable(cmap=cmap)

        # Obtain linear color range
        color_range = sm.to_rgba(np.linspace(0, 1, 256))[:,0:3]    # color range RGBA => RGB
        color_range = (color_range*255.0).astype(np.uint8)         # [0,1] => [0,255]
        color_range = np.squeeze(np.dstack([color_range[:,2], color_range[:,1], color_range[:,0]]), 0)  # RGB => BGR

        # Apply colormap for each channel individually
        channels = [cv2.LUT(image, color_range[:,i]) for i in range(3)]
        return np.dstack(channels)

    def gray2color(self, img):
        """Changes the thermal gray scale image to color using palette."""
        #return cv2.applyColorMap(img, cv2.COLORMAP_INFERNO)
        #if cv2.__version__.startswith("4"):
        #    return cv2.applyColorMap(img, cv2.COLORMAP_INFERNO)
        #else:
        #    return self.apply_custom_colormap(img, 'viridis')
        return self.apply_custom_colormap(img, 'inferno')

    def img_callback(self, image_msg):
        #rospy.loginfo(image_msg.encoding)
        camera_img = self.bridge.imgmsg_to_cv2(image_msg, "mono8")
        h, w = camera_img.shape[:2]
        xmin = 0
        ymin = 0
        xmax = w
        ymax = h 
        if (not np.isnan(self.xmin)):
            xmin = max(0, self.xmin)
        if (not np.isnan(self.xmax)):
            xmax = min(w, self.xmax)
        if (not np.isnan(self.ymin)):
            ymin = max(0,self.ymin)
        if (not np.isnan(self.ymax)):
            ymax = min(h, self.ymax)
        if (xmin > xmax):
            xmin,xmax = xmax,xmin
        if (ymin > ymax):
            ymin,ymax = ymax,ymin
        if (xmax-xmin <= 1):
            if (xmax < w):
                xmax = xmax + 1
            else:
                xmin = xmin - 1
        if (ymax-ymin <= 1):
            if (ymax < h):
                ymax = ymax + 1
            else:
                ymin = ymin - 1
        image_roi = camera_img[ymin: ymax, xmin: xmax]
        self.publish(self.gray2color(image_roi))

    def publish(self, image):
        self.pub_color.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    thermal2color = Thermal2Color()
    thermal2color.run()
