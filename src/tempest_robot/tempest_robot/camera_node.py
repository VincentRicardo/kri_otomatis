import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
import threading
import time
import cv2
import numpy as np
from ultralytics import YOLO
import math 



class Camera(object):
    """
    Base Camera object
    """

    def __init__(self):
        self._cam = None
        self._frame = None
        self._frame_width = None
        self._frame_height = None
        self._ret = False

        self.auto_undistortion = False
        self._camera_matrix = None
        self._distortion_coefficients = None

        self._is_running = False

    def _init_camera(self):
        """
        This is the first for creating our camera
        We should override this!
        """

        pass

    def start_camera(self):
        """
        Start the running of the camera, without this we can't capture frames
        Camera runs on a separate thread so we can reach a higher FPS
        """

        self._init_camera()
        self._is_running = True
        threading.Thread(target=self._update_camera, args=()).start()

    def _read_from_camera(self):
        """
        This method is responsible for grabbing frames from the camera
        We should override this!
        """

        if self._cam is None:
            raise Exception("Camera is not started!")

    def _update_camera(self):
        """
        Grabs the frames from the camera
        """

        while True:
            if self._is_running:
                self._ret, self._frame = self._read_from_camera()
            else:
                break

    def get_frame_width_and_height(self):
        """
        Returns the width and height of the grabbed images
        :return (int int): width and height
        """

        return self._frame_width, self._frame_height

    def read(self):
        """
        With this you can grab the last frame from the camera
        :return (boolean, np.array): return value and frame
        """
        return self._ret, self._frame

    def release_camera(self):
        """
        Stop the camera
        """

        self._is_running = False

    def is_running(self):
        return self._is_running

    def set_calibration_matrices(self, camera_matrix, distortion_coefficients):
        self._camera_matrix = camera_matrix
        self._distortion_coefficients = distortion_coefficients

    def activate_auto_undistortion(self):
        self.auto_undistortion = True

    def deactivate_auto_undistortion(self):
        self.auto_undistortion = False

    def _undistort_image(self, image):
        if self._camera_matrix is None or self._distortion_coefficients is None:
            import warnings
            warnings.warn("Undistortion has no effect because <camera_matrix>/<distortion_coefficients> is None!")
            return image

        h, w = image.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self._camera_matrix,
                                                               self._distortion_coefficients, (w, h),
                                                               1,
                                                               (w, h))
        undistorted = cv2.undistort(image, self._camera_matrix, self._distortion_coefficients, None,
                                    new_camera_matrix)
        return undistorted


class WebCamera(Camera):
    """
    Simple Webcamera
    """

    def __init__(self, video_src=0):
        """
        :param video_src (int): camera source code (it should be 0 or 1, or the filename)
        """

        super().__init__()
        self._video_src = video_src

    def _init_camera(self):
        super()._init_camera()
        self._cam = cv2.VideoCapture(self._video_src, cv2.CAP_V4L)
        self._ret, self._frame = self._cam.read()
        if not self._ret:
            raise Exception("No camera feed")
        self._frame_height, self._frame_width, c = self._frame.shape
        return self._ret

    def _read_from_camera(self):
        super()._read_from_camera()
        self._ret, self._frame = self._cam.read()   
        if self._ret:
            if self.auto_undistortion:
                self._frame = self._undistort_image(self._frame)
            return True, self._frame
        else:
            return False, None

    def release_camera(self):
        super().release_camera()
        self._cam.release()



class Image(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.talking_one = self.create_publisher(Int32MultiArray, "/cam_", 1)
        self.timer1_ = self.create_timer(0.01, self.camera_read)

        self.subscriber_flag = self.create_subscription(String, "/flag", self.kirim_, 1)

        # self.start_time = time.time()
        # self.display_time = 2
        # self.fc = 0
        # self.FPS = 0
        self.model = YOLO('yolov8_160_20.pt')
        
        self.webcam = WebCamera(video_src=0)
        self.webcam.start_camera()
        self.frame_width, self.frame_height = self.webcam.get_frame_width_and_height()

        self.center_x = int(self.frame_width // 2)
        self.center_y = int(self.frame_height // 2)
        self.distance = 0
        self.angle = 0
    
    def camera_read(self):
        # self.fc+=1
        # TIME = time.time() - self.start_time
        # if (TIME) >= self.display_time :
        #     self.FPS = self.fc / (TIME)
        #     self.fc = 0
        #     start_time = time.time()

        # fps_disp = "FPS: "+str(self.FPS)[:5]

        ret, frame = self.webcam.read()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self.model(frame, conf=0.8)[0]

        # for r in results:
        #     frame_gray = r.plot()

        for r in results:
            boxes = r.boxes
            for box in boxes :
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cls = int(box.cls)

                camera_width = x2 - x1
                self.distance = (10 *self.frame_width)/ camera_width

                self.angle = math.degrees(math.atan2(self.center_x - ((x1+x2)/2 ), self.distance))

                self.distance = round(self.distance,2)
                self.angle = round(self.angle, 2)

                #self.get_logger().info(str(distance))
                #self.get_logger().info(str(angle))

                cv2.putText(frame_gray, "D: {:.2f} cm".format(self.distance),(x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
                cv2.putText(frame_gray, "A: {:.2f} degrees".format(self.angle), (x1,y1-25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0),2)
                cv2.rectangle(frame_gray,(x1,y1),(x2,y2),(255,0,25),3)
                

        #image = cv2.putText(frame_gray, fps_disp, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
  
        cv2.imshow("webcam test", frame_gray)
        cv2.waitKey(1)
    def kirim_(self, message: String):
        message = Int32MultiArray()
        message.data = [self.distance, self.angle]
        self.talking_one.publish(message)

   
            

def main(args=None):
    rclpy.init(args=args)
    node = Image()
    rclpy.spin(node)
    rclpy.shutdown()