import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import os
import cv2
from datetime import datetime
import message_filters
import numpy as np
from detect import Detection
import line_profiler
from graspnetAPI import GraspGroup, Grasp
from scipy.spatial.transform import Rotation as R


class PiperPoseDetection(Node):
    def __init__(self):
        super().__init__("piper_pose_detection")
        self.cv_bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber(
            self, Image, "/camera/color/image_raw"
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_raw"
        )

        # Synchronize timestamps
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 3, 0.02
        )
        self.sync.registerCallback(self.img_callback)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 1
        )
        self.camera_info = {}
        self.piper_pose_detection_pub = self.create_publisher(
            PoseStamped, "/piper_pose_detection/pose_stamped", 10
        )

        self.img_path = "images/"
        self.depth_threshold = 0.5 * 1000  # m

        self.detection = Detection(
            checkpoint_path=os.path.join(
                os.path.dirname(__file__), "../logs/log_kn/checkpoint.tar"
            )
        )
        self.last_time = datetime.now().timestamp()
        self.save_as_file = True  # 是否保存为文件

    @line_profiler.profile
    def img_callback(self, ros_rgb_image: Image, ros_depth_image: Image):
        self.get_logger().debug("📸 收到新color图像消息，开始处理...")
        mask, np_color_image, np_depth_image = self.msg2img_mask(
            ros_rgb_image, ros_depth_image
        )
        # rotate
        # np_color_image = cv2.rotate(np_color_image, cv2.ROTATE_180)
        # np_depth_image = cv2.rotate(np_depth_image, cv2.ROTATE_180)
        # mask = cv2.rotate(mask.astype(np.uint8), cv2.ROTATE_180)
        # mask[:, 400:] = False
        # # 定义深度图的尺寸
        # height, width = np_depth_image.shape[:2]

        # # 定义高斯分布参数
        # A = 255  # 振幅（峰值）
        # x0, y0 = width // 2, height // 2  # 高斯分布的中心
        # sigma_x, sigma_y = 100, 100  # 标准差，控制钟形曲线的宽度

        # # 创建二维网格
        # x = np.arange(0, width)
        # y = np.arange(0, height)
        # X, Y = np.meshgrid(x, y)

        # # 计算二维高斯分布
        # np_depth_image = A * np.exp(
        #     -((X - x0) ** 2 / (2 * sigma_x**2) + (Y - y0) ** 2 / (2 * sigma_y**2))
        # )
        if self.save_as_file:
            # now = datetime.now().strftime("%Y%m%d_%H%M%S")
            now = 0  # For testing purposes, use a fixed timestamp
            os.makedirs(self.img_path, exist_ok=True)

            mask_path = os.path.join(self.img_path, f"mask_{now}.jpg")
            cv2.imwrite(
                mask_path, mask.astype(np.uint8) * 255
            )  # Convert boolean mask to uint8
            self.get_logger().debug(f"✅ mask图像存于 {mask_path}")

            color_path = os.path.join(self.img_path, f"color_{now}.jpg")
            cv2.imwrite(color_path, np_color_image)
            self.get_logger().debug(f"✅ color图像存于 {color_path}")

            depth_path = os.path.join(self.img_path, f"depth_{now}.jpg")
            # normalize depth image to 0-255 for visualization
            np_depth_image_norm = cv2.normalize(
                np_depth_image, None, 0, 255, cv2.NORM_MINMAX
            ).astype(np.uint8)
            cv2.imwrite(depth_path, np_depth_image_norm)
            self.get_logger().debug(f"✅ depth图像存于 {depth_path}")

        intrinsic = self.camera_info.get("k", None)
        if intrinsic is not None:
            intrinsic = np.array(intrinsic).reshape(3, 3)
            gg_path = os.path.join(self.img_path, f"grasps_{now}.png")
            self.get_logger().debug(f"相机内参: {intrinsic}")
            gg, cloud = self.detection.detect_grasps(
                color=np_color_image,
                depth=np_depth_image,
                intrinsic=intrinsic,
                workspace_mask=mask,
            )
            if gg is not None and len(gg) != 0:
                # 非极大值抑制
                gg.nms(translation_thresh=0.1, rotation_thresh=45 * np.pi / 180.0)
                gg.sort_by_score()
                grasp = gg[0]
                print(grasp)
                self.piper_pose_detection_pub.publish(
                    PiperPoseDetection.grasp2msg(grasp, ros_rgb_image.header.stamp)
                )
                self.detection.draw_grasps(
                    gg[:1], cloud, gg_path, np_depth_image.shape[:2]
                )

        now = datetime.now().timestamp()
        print(
            f"处理耗时: {now - self.last_time:.2f}秒，帧率: {1 / (now - self.last_time):.2f} FPS"
        )
        self.last_time = now

    def msg2img_mask(self, color_img_msg: Image, depth_img_msg: Image):
        """
        parameters
        color_img_msg: Message class 'Image'--RGB image
        depth_img_msg: Message class 'Image'--Z16 image

        returns
        image and mask: np.ndarray, np.ndarray
        """
        if color_img_msg is None or depth_img_msg is None:
            self.get_logger().error(
                "bg_removal: Background removal error, color or depth msg was None"
            )
            return None, None, None

        # Convert color image msg to opencv gbr8 image
        cv_color_image = self.cv_bridge.imgmsg_to_cv2(
            color_img_msg, desired_encoding="bgr8"
        )
        np_color_image = np.array(cv_color_image, dtype=np.uint8)

        # Convert depth image msg
        cv_depth_image = self.cv_bridge.imgmsg_to_cv2(
            depth_img_msg, desired_encoding="passthrough"
        )
        np_depth_image = np.array(cv_depth_image, dtype=np.uint16)

        mask = (np_depth_image < self.depth_threshold) & (np_depth_image != 0)
        return mask, np_color_image, np_depth_image

    def camera_info_callback(self, msg: CameraInfo):
        """
        Get camera parameters through a callback function.
        """
        self.camera_info["k"] = msg.k
        self.camera_info["p"] = msg.p
        self.camera_info["d"] = msg.d
        self.camera_info["r"] = msg.r
        self.camera_info["width"] = msg.width
        self.camera_info["height"] = msg.height
        self.camera_info["roi"] = msg.roi

        self.destroy_subscription(self.camera_info_sub)

    @staticmethod
    def grasp2msg(grasp: Grasp, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        trans = grasp.translation.astype(float)
        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]
        quat = R.from_matrix(grasp.rotation_matrix.reshape(3, 3)).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg


def main(args=None):
    # ros2 launch orbbec_camera dabai_dcw.launch.py depth_registration:=true enable_d2c_viewer:=true
    rclpy.init(args=args)
    node = PiperPoseDetection()
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
