import os
import sys
import numpy as np
import open3d as o3d
import importlib
import scipy.io as scio
from PIL import Image
import line_profiler

import torch
from graspnetAPI import GraspGroup, Grasp

GRASPNET_ROOT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "graspnet_baseline"
)
sys.path.append(os.path.join(GRASPNET_ROOT_DIR, "models"))
sys.path.append(os.path.join(GRASPNET_ROOT_DIR, "utils"))

from graspnet_baseline.models.graspnet import GraspNet, pred_decode
from graspnet_baseline.utils.collision_detector import ModelFreeCollisionDetector
from graspnet_baseline.utils.data_utils import (
    CameraInfo,
    create_point_cloud_from_depth_image,
)


class Detection:

    def __init__(
        self,
        checkpoint_path: str,
        num_point: int = 200000,
        num_view: int = 300,
        collision_thresh: float = 0.01,
        voxel_size: float = 0.01,
    ):
        """
        Initialize the Detection class with model parameters.
        Args:
            checkpoint_path (str): Path to the model checkpoint.
            num_point (int): Number of points to sample from the point cloud.
            num_view (int): Number of views for the model.
            collision_thresh (float): Threshold for collision detection.
            voxel_size (float): Voxel size for processing point clouds.
        """
        self.checkpoint_path = checkpoint_path
        self.num_point = num_point
        self.num_view = num_view
        self.collision_thresh = collision_thresh
        self.voxel_size = voxel_size

        self.net = self.get_net()

    def get_net(self):
        # Init the model
        net = GraspNet(
            input_feature_dim=0,
            num_view=self.num_view,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        # Load checkpoint
        checkpoint = torch.load(self.checkpoint_path)
        net.load_state_dict(checkpoint["model_state_dict"])
        start_epoch = checkpoint["epoch"]
        print(
            "-> loaded checkpoint %s (epoch: %d)" % (self.checkpoint_path, start_epoch)
        )
        # set model to eval mode
        net.eval()
        return net

    @line_profiler.profile
    def _get_cloud(
        self,
        color: np.ndarray,
        depth: np.ndarray,
        intrinsic: np.ndarray,
        factor_depth: float,
        workspace_mask: np.ndarray,
    ):
        # downsample color and depth
        # color = color[::4, ::4, :]
        # depth = depth[::4, ::4]
        # generate cloud
        camera = CameraInfo(
            depth.shape[1],
            depth.shape[0],
            intrinsic[0][0],
            intrinsic[1][1],
            intrinsic[0][2],
            intrinsic[1][2],
            factor_depth,
        )
        mask = (workspace_mask & (depth > 0)).astype(np.bool_)
        if not np.any(mask):
            return None, None
        cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
        # get valid points
        cloud_masked = cloud[mask]
        color_masked = color[mask]

        # sample points
        if len(cloud_masked) >= self.num_point:
            idxs = np.random.choice(len(cloud_masked), self.num_point, replace=False)
        else:
            idxs1 = np.arange(len(cloud_masked))
            idxs2 = np.random.choice(
                len(cloud_masked), self.num_point - len(cloud_masked), replace=True
            )
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]

        # convert data
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
        end_points = dict()
        cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cloud_sampled = cloud_sampled.to(device)
        end_points["point_clouds"] = cloud_sampled
        end_points["cloud_colors"] = color_sampled

        return end_points, cloud

    def _get_and_process_data(self, data_dir):
        # load data
        color = (
            np.array(Image.open(os.path.join(data_dir, "color.png")), dtype=np.float32)
            / 255.0
        )
        depth = np.array(Image.open(os.path.join(data_dir, "depth.png")))
        workspace_mask = np.array(
            Image.open(os.path.join(data_dir, "workspace_mask.png"))
        )
        meta = scio.loadmat(os.path.join(data_dir, "meta.mat"))
        intrinsic = meta["intrinsic_matrix"]
        factor_depth = meta["factor_depth"]

        return self.get_cloud(color, depth, intrinsic, factor_depth, workspace_mask)

    def _get_grasps(self, end_points):
        # Forward pass
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)
        gg.nms()
        gg.sort_by_score()
        return gg

    def _collision_detection(self, gg, cloud):
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(
            gg, approach_dist=0.05, collision_thresh=self.collision_thresh
        )
        gg = gg[~collision_mask]
        return gg

    @line_profiler.profile
    def detect_grasps(
        self,
        color: np.ndarray,
        depth: np.ndarray,
        intrinsic: np.ndarray,
        factor_depth: float = 1000,  # mm
        workspace_mask: np.ndarray = True,
    ):
        """
        Detect grasps from color and depth images.
        Args:
            net (GraspNet): The grasp detection model.
            color (np.ndarray): Color image in RGB format.
            depth (np.ndarray): Depth image.
            intrinsic (np.ndarray): Camera intrinsic matrix.
            factor_depth (float): Factor to scale depth values.
        Returns:
            GraspGroup: Detected grasps.
        """
        endpoints, cloud = self._get_cloud(
            color, depth, intrinsic, factor_depth, workspace_mask
        )
        if endpoints is None or cloud is None:
            return None, None
        gg = self._get_grasps(endpoints)
        if self.collision_thresh > 0:
            gg = self._collision_detection(gg, np.array(cloud.points))
        return gg, cloud

    @staticmethod
    def draw_grasps(gg: GraspGroup, cloud, path: str, size: np.ndarray):
        # gg.add(
        #     Grasp(
        #         1,
        #         0.06,
        #         0.02,
        #         0.02,
        #         np.array([1, 0, 0, 0, 1, 0, 0, 0, 1]),
        #         [0, 0, 1],
        #         -1,
        #     )
        # )
        # gg.add(
        #     Grasp(
        #         0.1,
        #         0.06,
        #         0.02,
        #         0.02,
        #         gg.rotation_matrices[0],
        #         [0, 0, 0.5],
        #         -1,
        #     )
        # )
        grippers = gg.to_open3d_geometry_list()
        # o3d.visualization.draw_geometries([cloud, *grippers])

        # 使用 Open3D 的渲染模块
        scene = o3d.visualization.rendering.OffscreenRenderer(
            size[1], size[0]
        )  # 设置渲染尺寸
        scene.scene.set_background([1, 1, 1, 1])  # 设置背景为白色

        # # 创建一个坐标轴对象
        # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        #     size=0.1, origin=[0, 0, 0]  # 坐标轴的大小  # 坐标轴的原点
        # )
        # # 将坐标轴添加到场景中
        # scene.scene.add_geometry(
        #     "coordinate_frame",
        #     coordinate_frame,
        #     o3d.visualization.rendering.MaterialRecord(),
        # )

        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultLit"  # 或者 "unlit" 也可以
        material.point_size = 0.5  # 设置点大小（适用于点云）
        cloud.paint_uniform_color([0.0, 0.0, 1.0])  # 设置点云为蓝色
        scene.scene.add_geometry("cloud", cloud, material)
        for i, gripper in enumerate(grippers):
            scene.scene.add_geometry(
                f"gripper_{i}", gripper, o3d.visualization.rendering.MaterialRecord()
            )
        # 画小球作为参考
        # sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
        # sphere.paint_uniform_color([1, 0, 0])  # 红色
        # sphere.translate([0, 0, 0.5])  # 放在哪点
        # scene.scene.add_geometry(
        #     "sphere", sphere, o3d.visualization.rendering.MaterialRecord()
        # )
        # 设置相机视角
        center = [0, 0.0, 0.5]  # 看向
        eye = [0, 0, 0.05]  # 相机在的位置
        up = [0, -1, 0]
        scene.scene.camera.look_at(center, eye, up)

        # 渲染并保存为图片
        image = scene.render_to_image()
        o3d.io.write_image(path, image)
