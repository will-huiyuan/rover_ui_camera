import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer

class CameraSubscriber():
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_subscriber = rospy.Subscriber("/camera_frames", Image, self.camera_callback)
        self.image = None

    def camera_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        q_image = QImage(rgb_image.data, rgb_image.shape[1], rgb_image.shape[0], QImage.Format_RGB888)
        self.image = QPixmap.fromImage(q_image)

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super(MainWindow, self).__init__()
        self.camera_subscriber = CameraSubscriber()

        self.label = QtWidgets.QLabel(self)
        self.setCentralWidget(self.label)

        self.timer_camera = QTimer()
        self.timer_camera.timeout.connect(self.show_camera)
        self.timer_camera.start(30)

    def show_camera(self):
        if self.camera_subscriber.image is not None:
            self.label.setPixmap(self.camera_subscriber.image)

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    rospy.init_node('camera_subscriber_node')
    window = MainWindow()
    window.show()
    app.exec_()
