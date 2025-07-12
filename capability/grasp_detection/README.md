# 夹爪位姿检测

提供夹爪位姿检测的功能，使用 GraspNet 模型。

* 输入：深度相机模块`/camera/color/image_raw`, `/camera/color/image_raw`, `/camera/color/camera_info`
* 输出：夹爪位姿检测结果（相机坐标系）`/piper_pose_detection/pose_stamped`

## 安装

安装步骤见 [https://github.com/graspnet/graspnet-baseline 的README.md](https://github.com/graspnet/graspnet-baseline/blob/main/README.md)。

需要修改graspAPI/setup.py中的sklearn为scikit-learn。

需要nvcc编译，注意cuda版本要求。在cuda12.2，torch2.5.1+cu121复现成功。cuda11.5好像不行。
