from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from threading import Lock


class FetchPoseFromROS(Node):
    """
    Fetches the target pose from ROS.
    """

    def __init__(self, callback=None):
        super().__init__("piper_control_fetch_pose_from_ros")
        # TODO: 可能要从tf获取转换后的目标位姿
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            "/piper_pose_detection/pose_stamped",
            self.target_pose_callback,
            10,
        )
        self.target_position = None
        self.target_euler = None
        if callback is not None:
            if not callable(callback):
                raise TypeError("Callback must be a callable function.")
            self.callback = callback
        else:
            self.callback = lambda pos, euler: print(
                f"Target Position: {pos}, Target Euler: {euler}"
            )
        self.lock = Lock()

    def target_pose_callback(self, msg: PoseStamped):
        self.target_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        self.target_euler = R.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        ).as_euler("zyx", degrees=True)
        with self.lock:
            self.callback(self.target_position, self.target_euler)

    def get_pose(self):
        """
        Returns the target position and euler angles.
        :return: target_position, target_euler
        """
        return self.target_position, self.target_euler


if __name__ == "__main__":
    from threading import Thread
    import rclpy

    rclpy.init()
    fetch_pose_node = FetchPoseFromROS()
    t = Thread(target=rclpy.spin, args=(fetch_pose_node,))
    t.start()
    # while True:
    #     position, euler = fetch_pose_node.get_pose()
    #     if position is not None and euler is not None:
    #         print(f"Target Position: {position}, Target Euler: {euler}")
    t.join()
    fetch_pose_node.destroy_node()
    rclpy.shutdown()
