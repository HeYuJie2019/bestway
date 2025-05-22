import open3d as o3d
import numpy as np

def main():
    # 读取 PCD 文件
    pcd_file = "path/to/your/pointcloud.pcd"  # 替换为你的 PCD 文件路径
    pcd = o3d.io.read_point_cloud(pcd_file)
    print("PCD 文件加载成功！")

    # 显示原始点云
    o3d.visualization.draw_geometries([pcd], window_name="原始点云")

    # 创建一条路径（示例：直线路径）
    path_points = np.array([
        [0, 0, 0],  # 起点
        [1, 1, 1],  # 中间点
        [2, 2, 2]   # 终点
    ])
    path_lines = [[0, 1], [1, 2]]  # 定义路径的线段
    colors = [[1, 0, 0] for _ in range(len(path_lines))]  # 路径颜色为红色

    # 创建 LineSet 对象表示路径
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(path_points)
    line_set.lines = o3d.utility.Vector2iVector(path_lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # 显示点云和路径
    o3d.visualization.draw_geometries([pcd, line_set], window_name="点云与路径")

if __name__ == "__main__":
    main()