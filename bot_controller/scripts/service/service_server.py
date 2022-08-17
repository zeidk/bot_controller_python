import cv2
import rospy
from sensor_msgs.msg import Image
# ROS library that provides an interface between ROS and OpenCV
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.srv import TakePicture, TakePictureResponse

class ServiceServer(object):
    """Class which provides a Service server to handle Service client requests.


    Args:
        _bridge (CvBridge): OpenCV bridge object.
        _image (cv::Mat): OpenCV image converted from a ROS image.
    """
    
    _service_name = 'take_picture_service'
    
    @classmethod
    def service_name(cls):
        """Class method which returns the name of the service

        Returns:
            str: The name of the Service
        """
        return cls._service_name
    

    def __init__(self):
        self._bridge = CvBridge()
        self._image = None
        rospy.init_node('camera_server')
        # Subscribe to the turtlebot camera Topic
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        # Callback to process client's request
        rospy.Service(self._service_name, TakePicture, self.service_cb)
        rospy.loginfo("Server is ready to accept request")
        rospy.spin()


    def camera_cb(self, image_message):
        """
        Callback to handle Messages published on the Topic /camera/rgb/image_raw.

        Args:
            image_message (Image): Image data from Turtlebot camera.
        """

        # Convert image to OpenCV format
        cv_image = None
        try:
            cv_image = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # numpy array
        self._image = cv_image


    def service_cb(self, request):
        """
        Callback to process a Service Client's request.

        Args:
            request (str): Name of the picture to save camera image on disk.

        Returns:
            str: Whether or not the image was saved on disk.
        """
        if self._image is not None:
            rospy.loginfo("Processing request")
            # location to save the image
            final_image = "/tmp/"+str(request.picture_name)+'.jpg'
            # save image on disk
            write_status = cv2.imwrite(final_image, self._image)
            if write_status:
                return TakePictureResponse("Image saved at {}".format(final_image))
            else:
                return TakePictureResponse("Image NOT saved")
