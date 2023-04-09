import numpy as np
import pyrealsense2 as rs
import rospy
import rospy.names
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class RealsenseD435i():
    def __init__(self):
        self.gyro = None
        self.accel = None
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        self.device = pipeline_profile.get_device()

        # setting the stream
        self.config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
        self.img_pub = rospy.Publisher("/realsense/rgb/image", Image, queue_size=1)

    def ros_pubilsh_image(self, image_np):
        img_msg = self.bridge.cv2_to_imgmsg(cvim=image_np, encoding="passthrough")
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "camera"
        self.img_pub.publish(img_msg)

    def run(self):
        # Start streaming
        pipeline_profile = self.pipeline.start(self.config)
        device = pipeline_profile.get_device()
        depth_sensor = device.query_sensors()[0]
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)

        while True:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            self.ros_pubilsh_image(color_image)

            if rospy.is_shutdown():
                break

        self.pipeline.stop()


if __name__ == '__main__':
    rospy.init_node("realsensed435i")
    realsense_d435i = RealsenseD435i()
    realsense_d435i.run()
