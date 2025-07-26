from fetch_pose import from_ros
from piper_control import ctrl_by_piper_sdk
import rclpy
import time
from builtin_interfaces.msg import Time
from threading import Lock


def main():
    control = ctrl_by_piper_sdk.CtrlByPiperSDK(debug_mode=False)
    control.reset(move_mode_end_pose=False)
    print(control.get_joint())
    control.reset(move_mode_end_pose=True)
    control.set_ee_pose(position=[0.0, 0.0, 0.2], euler_angles=[0.0, 90.0, 0.0])

    last_time = time.time()
    lock = Lock()

    def move_callback(position, euler, timestamp: Time):
        nonlocal last_time, lock
        nonlocal control
        now = time.time()
        if not lock.acquire(blocking=False):
            print(
                f"Target Position: {[round(n,2) for n in position]}, Target Euler: {[round(float(n),2) for n in euler]}"
            )
            control.set_ee_pose(position=[0.0, 0.0, 0.2], euler_angles=[0.0, 90.0, 0.0])
            control.set_ee_pose(position=position, euler_angles=[0, 90, 0])
            control.set_ee_pose(position=[0.0, 0.0, 0.2], euler_angles=[0.0, 90.0, 0.0])
            last_time = now
            lock.release()

    rclpy.init()
    fetch_pose_node = from_ros.FetchPoseFromROS(move_callback)
    rclpy.spin(fetch_pose_node)
    fetch_pose_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
