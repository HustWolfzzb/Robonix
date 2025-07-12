from fetch_pose import from_ros
from piper_control import ctrl_by_piper_sdk
import rclpy


def main():
    control = ctrl_by_piper_sdk.CtrlByPiperSDK()
    control.reset(move_mode_end_pose=True)
    control.set_ee_pose(position=[0.0, 0.0, 0.2], euler_angles=[0.0, 90.0, 0.0])

    def move_callback(position, euler):
        print(f"Target Position: {position}, Target Euler: {euler}")
        # control.set_ee_pose(position=position, euler_angles=euler)

    rclpy.init()
    fetch_pose_node = from_ros.FetchPoseFromROS(move_callback)
    rclpy.spin(fetch_pose_node)
    fetch_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
