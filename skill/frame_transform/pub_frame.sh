# 格式: x y z qx qy qz qw frame_id child_frame_id
# 这是一个示例值，请替换为您自己的标定结果
# 两者坐标系定义不同，其旋转矩阵：
# [[0,0,1]
#  [-1,0,0]
#  [0,-1,0]]
# 四元数：0.5 -0.5 0.5 -0.5
ros2 run tf2_ros static_transform_publisher -0.05 0.0 0.1 0.5 -0.5 0.5 -0.5 base_link camera_color_optical_frame
