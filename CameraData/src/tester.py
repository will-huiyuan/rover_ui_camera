import os, sys
from pydoc_data.topics import topics
from cv_bridge import CvBridge

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
sys.path.append("/home/will/catkin_ws/src/rover")
import rospy
import cv2
import camera_capture
import UI_App.src.main as uimain
from PyQt5.QtWidgets import QApplication, QWidget

video = cv2.VideoCapture("UI_App/src/test_video.mp4")
flag,img = video.read()
def openCV_to_ros_image(cv_image):
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    return ros_image
def ros_to_openCV_image(ros_image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
    return cv_image

app = QApplication(sys.argv)

w = QWidget()
while True:
    img = cv2.imread("CameraData/src/Jellyfish.jpg")
    rosimg = openCV_to_ros_image(img)
    RtoCimg = ros_to_openCV_image(rosimg)

    cv2.imshow('w', RtoCimg)
    app.exec()


# def ros_to_openCV_image(self, ros_image):
#         bridge = CvBridge()
#         cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
#         return cv_image

# def show_camera(self):
#     self.frames = self.ros_to_openCV_image(self.cam_image)
#     self.frames = cv2.imdecode(self.frames, 1)
#     show = cv2.cvtColor(self.frames, cv2.COLOR_BGR2RGB)  # change color into RGB
#     showImage = QtGui.QImage(show.data, show.shape[1], show.shape[0],
#                                 QtGui.QImage.Format_RGB888)  # Turn the read video data into QImage form
#     # showImage = self.ros_to_openCV_image(self.cam_image)
    # self.Camera.setPixmap(QtGui.QPixmap.fromImage(showImage))
# uimain.main()
# rospy.spin()
