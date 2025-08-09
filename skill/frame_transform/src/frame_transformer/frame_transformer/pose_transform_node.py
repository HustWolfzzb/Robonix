import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from rclpy.duration import Duration


class PoseTransformerNode(Node):
    def __init__(self):
        super().__init__("pose_transformer_node")

        # 声明参数，使其更具通用性
        self.declare_parameter("target_frame", "base_link")
        self.target_frame = (
            self.get_parameter("target_frame").get_parameter_value().string_value
        )

        # TF2 初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅者，订阅来自GraspNet的位姿
        self.pose_subscriber = self.create_subscription(
            PoseStamped, "/piper_pose_detection/pose_stamped", self.pose_callback, 10
        )

        # 发布者，发布转换后的位姿
        self.transformed_pose_publisher = self.create_publisher(
            PoseStamped, "/piper_base_link/pose_stamped", 10
        )

        self.get_logger().info(f"坐标转换节点已启动，目标坐标系: '{self.target_frame}'")

    def pose_callback(self, msg: PoseStamped):
        source_frame = msg.header.frame_id
        self.get_logger().info(
            f"收到来自 '{source_frame}' 的位姿[{self.pose2str(msg)}]，准备转换到 '{self.target_frame}'"
        )

        try:
            # 查找坐标变换
            # 我们等待变换可用，设置一个超时时间
            when = self.get_clock().now() - Duration(seconds=1.0)  # 容忍1秒内的旧变换
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                when,  # 使用稍早的时间戳来避免等待最新的变换
                timeout=Duration(seconds=1.0),
            )

            # 使用tf2_geometry_msgs进行坐标转换
            transformed_pose_stamped = do_transform_pose_stamped(msg, transform)

            # 发布转换后的位姿
            self.transformed_pose_publisher.publish(transformed_pose_stamped)
            self.get_logger().info(
                f"成功转换位姿[{self.pose2str(transformed_pose_stamped)}]并发布到 '{self.target_frame}'"
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().error(f"坐标转换失败: {e}")

    @staticmethod
    def pose2str(pose: PoseStamped) -> str:
        return (
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}), "
            f"({pose.pose.orientation.x:.2f}, {pose.pose.orientation.y:.2f}, "
            f"{pose.pose.orientation.z:.2f}, {pose.pose.orientation.w:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
