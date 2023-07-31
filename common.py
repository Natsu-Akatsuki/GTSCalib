import logging
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import yaml
from ampcl import visualization
from ampcl.visualization import o3d_viewer_from_pointcloud
from easydict import EasyDict
from scipy import signal, stats
from scipy.spatial.transform import Rotation
from sklearn.decomposition import PCA
from sklearn.mixture import GaussianMixture


class Config:
    def __init__(self, user_cfg_file):
        cfg_dict = {}
        with open(user_cfg_file, 'r') as f:
            user_cfg = yaml.load(f, Loader=yaml.FullLoader)
        with open("config/developer/developer.yaml", 'r') as f:
            developer_cfg = yaml.load(f, Loader=yaml.FullLoader)
        with open("config/developer/lidar.yaml", 'r') as f:
            sensor_cfg = yaml.load(f, Loader=yaml.FullLoader)

        cfg_dict.update(user_cfg)
        cfg_dict.update(developer_cfg)
        cfg_dict.update(sensor_cfg)
        cfg_dict = EasyDict(cfg_dict)

        # debug parameter
        self.vis_raw_chessboard_pc = cfg_dict.Debug.visualize_raw_chessboard_pointcloud
        self.vis_plane_fitting_result = cfg_dict.Debug.visualize_plane_fitting_result
        self.vis_proj_chessboard_pc = cfg_dict.Debug.visualize_proj_chessboard_pointcloud
        self.vis_intensity_threshold = cfg_dict.Debug.visualize_intensity_threshold
        self.vis_binary_chessboard_pc = cfg_dict.Debug.visualize_binary_chessboard_pointcloud
        self.vis_ideal_model_chessboard_corners = cfg_dict.Debug.visualized_ideal_model_chessboard_corners
        self.vis_pca_pc = cfg_dict.Debug.visualize_pca_pointcloud
        self.vis_2D_corners = cfg_dict.Debug.visualize_2D_corners
        self.vis_3D_corners_in_lidar_frame = cfg_dict.Debug.visualize_3D_corners_in_lidar_frame
        self.vis_contour = cfg_dict.Debug.visualize_contour
        dataset_dir = cfg_dict.Dataset.DataDir
        self.dataset_dir = str(Path(dataset_dir).resolve())

        # chessboard parameter
        self.corner_num_along_left_side = cfg_dict.ChessboardParam.CornerNumAlongLeftSide
        self.corner_num_along_bottom_side = cfg_dict.ChessboardParam.CornerNumAlongBottomSide
        if self.corner_num_along_bottom_side > self.corner_num_along_left_side:
            self.is_bottom_side_long = True
        elif self.corner_num_along_bottom_side < self.corner_num_along_left_side:
            self.is_bottom_side_long = False
        else:
            raise NotImplemented
        self.grid_size = cfg_dict.ChessboardParam.GridSize
        self.chessboard_pattern = (self.corner_num_along_left_side, self.corner_num_along_bottom_side)
        self.corner_color = cfg_dict.ChessboardParam.CornerColor

        # sensor parameter
        lidar = cfg_dict.SensorParam.LiDAR
        self.SensorParam = cfg_dict.LiDARParam[lidar]

        # algorithm parameter
        self.plane_fitting_threshold = cfg_dict.AlgorithmParam.plane_fitting_threshold
        self.ransac_seed = cfg_dict.AlgorithmParam.ransac_seed
        self.voxel_grid_method = cfg_dict.AlgorithmParam.voxel_grid_method
        self.reset_segmentation = cfg_dict.AlgorithmParam.reset_segmentation
        self.boundary_threshold = cfg_dict.AlgorithmParam.boundary_threshold

        # other parameter
        self.use_hash_map_reproject = cfg_dict.OtherParam.use_hash_map_reproject
        self.use_plane_refine = cfg_dict.OtherParam.use_plane_refine

        # developer parameter
        self.pnp_method = cfg_dict.DeveloperParam.pnp_method
        self.binarization_methods = cfg_dict.DeveloperParam.binarization_methods


def ray_projection(plane_model, pointcloud, debug=False):
    """
    project the pointcloud onto ideal chessboard plane based on ray projection model
    :param debug:
    :param plane_model:
    :param pointcloud：pointcloud of chessboard
    :return: pointcloud：projected pointcloud of chessboard which on ideal chessboard
    """
    A, B, C, D = plane_model
    # 直线与平面交点的公式
    x0, y0, z0 = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]
    t = -D / (A * x0 + B * y0 + C * z0)
    pointcloud[:, :3] = t.reshape(-1, 1) * pointcloud[:, :3]

    if debug:
        o3d_viewer_from_pointcloud(pointcloud, is_show=True)
    return pointcloud


def find_intensity_threshold(intensity, methods, debug=False):
    """
    find the intensity threshold to classifying pointcloud based on intensity into two classes.
    :param pc:
    :param debug:
    :return:
    """

    # 用KDE拟合居中的intensity的分布
    intensity_uint8 = intensity.astype(np.uint8)

    for method in methods:
        if method == "KDE":
            pdf = stats.kde.gaussian_kde(intensity_uint8)
            x = np.arange(0, 256, 1)
            peak_x, _ = signal.find_peaks(pdf(x), distance=30)
            sorted_peak_x = peak_x[np.argsort(pdf(peak_x))][::-1]
            topk_peak_x = sorted_peak_x[:2]
            intensity_threshold = topk_peak_x.mean()
            logging.info(f"[intensity threshold based on KDE is]: {intensity_threshold}")
        elif method == "ACSC":
            gmm = GaussianMixture(n_components=2, covariance_type="diag",
                                  max_iter=10000,
                                  means_init=np.array([[15], [100]])).fit(intensity_uint8.reshape(-1, 1))
            intensity_threshold = gmm.means_[:2].mean().astype(np.uint8)
            logging.info(f"[intensity threshold based on ACSC's is]: {intensity_threshold}")
        elif method == "RCLC":
            gmm = GaussianMixture(n_components=2, covariance_type="diag",
                                  max_iter=10000,
                                  means_init=np.array([[15], [100]])).fit(intensity_uint8.reshape(-1, 1))
            black_mean = gmm.means_[0][0]
            white_mean = gmm.means_[1][0]
            black_std = np.sqrt(gmm.covariances_[0][0])
            white_std = np.sqrt(gmm.covariances_[1][0])

            intensity_threshold = (black_mean + 2 * black_std).astype(np.uint8)
            logging.info(f"[intensity threshold based on RCLC's is]: {intensity_threshold}")
        elif method == "ACSC+KDE":
            pdf = stats.kde.gaussian_kde(intensity_uint8)
            x = np.arange(0, 256, 1)
            peak_x, _ = signal.find_peaks(pdf(x), distance=20)
            sorted_peak_x = peak_x[np.argsort(pdf(peak_x))][::-1]
            topk_peak_x = sorted_peak_x[:2]
            gmm = GaussianMixture(n_components=2, covariance_type="diag",
                                  max_iter=10000,
                                  means_init=np.array([[topk_peak_x[0]], [topk_peak_x[1]]])).fit(
                intensity_uint8.reshape(-1, 1))
            intensity_threshold = gmm.means_[:2].mean().astype(np.uint8)
            logging.info(f"[intensity threshold based on GMM+KDE's is]: {intensity_threshold}")
        else:
            raise NotImplemented

    colors = np.zeros((151), dtype=np.uint8)
    colors[0:intensity_threshold.astype(np.uint8)] = 0
    colors[intensity_threshold.astype(np.uint8):151] = 1

    if debug:
        fig, ax = plt.subplots(dpi=500)
        matplot_color = np.zeros((151, 4))
        matplot_color[colors == 0] = [1, 0, 0, 0]
        matplot_color[colors == 1] = [0, 1, 0, 0]
        n, bins, patches = plt.hist(intensity_uint8, bins=151)
        for i in range(len(patches)):
            patches[i].set_fc('red' if colors[i] == 0 else 'green')
        plt.xlabel('Intensity')
        plt.ylabel('Frequency')
        plt.title('Histogram of Intensity')
        plt.xlim([0, 255])
        plt.show()

        # 强度概率密度函数
        plt.plot(x, pdf(x))
        plt.plot(topk_peak_x, pdf(topk_peak_x), 'o', markersize=8)
        plt.ylim([0, 0.025])
        plt.show()

    return intensity_threshold


def pca(pointcloud, debug=False, is_bottom_side_long=True):
    """
    将标定版点云从激光雷达系转换到标定版系下
    标定版系的定义：
    1）原点位置：为标定板点云的几何中心
    2）坐标系方向：规定标定板的x轴平行于侧边，标定版的底边平行于y轴（根据底边是长边还是短边判断底边）
    最大方差对应的奇异向量即y轴，次之为z轴，z轴为x轴和y轴的叉乘（满足右手坐标系）
    @ref: https://github.com/mfxox/ILCC/blob/c3f38e3aaa07b0d3981b33be277cccd48a2fb595/ILCC/pcd_corners_est.py#L598

    :param debug: 可视化标定板点云和标定板坐标系
    :param pointcloud_in_lidar_frame:
    :return:
    """
    intensity = pointcloud[:, 3]
    pointcloud_in_lidar_frame = pointcloud[:, :3]

    #                [ x ]
    # transform_mat  [ y ]
    #                [ z ]
    pca = PCA(n_components=3)
    pca.fit(pointcloud_in_lidar_frame)
    rmat_lidar_to_chessboard = np.zeros((3, 3), dtype=np.float32)
    if is_bottom_side_long:
        # 令标定板底边为x轴，侧边为y轴（又知底边为长边->x轴对应方差最大）
        rmat_lidar_to_chessboard[0], rmat_lidar_to_chessboard[1] = pca.components_[0], pca.components_[1]
    else:
        # 令标定板底边为x轴，侧边为y轴（又知底边为短边->x轴对应方差次大）
        rmat_lidar_to_chessboard[0], rmat_lidar_to_chessboard[1] = pca.components_[1], pca.components_[0]

    # 坐标系需满足右手系
    # 标定板的y轴需要跟激光雷达系的z轴的夹角应该为锐角
    if np.dot(rmat_lidar_to_chessboard[1], np.array([0, 0, 1])) < 0:
        rmat_lidar_to_chessboard[1] = -rmat_lidar_to_chessboard[1]

    rmat_lidar_to_chessboard[2] = np.cross(rmat_lidar_to_chessboard[0], rmat_lidar_to_chessboard[1])

    if np.dot(rmat_lidar_to_chessboard[2], 0 - pointcloud_in_lidar_frame.mean(axis=0)) < 0:
        rmat_lidar_to_chessboard[0] = -rmat_lidar_to_chessboard[0]
        rmat_lidar_to_chessboard[2] = -rmat_lidar_to_chessboard[2]

    # （坐标变换）将右边列向量表示的坐标数据转换到左边行向量所张成的空间
    # tf_lidar_to_chessboard @ (pointcloud_in_lidar.T)
    pc_in_chessboard_frame = pointcloud_in_lidar_frame @ rmat_lidar_to_chessboard.T
    tvec_in_chessboard_frame = pc_in_chessboard_frame.mean(axis=0)
    pc_in_chessboard_frame = pc_in_chessboard_frame - tvec_in_chessboard_frame
    pc_in_chessboard_frame = np.column_stack((pc_in_chessboard_frame, intensity))

    if debug:
        # 可视化标定板坐标系和其下的点云
        coordinate_point = np.array(
            [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
        lines = [[0, 1], [0, 2], [0, 3]]
        colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(coordinate_point),
                                        lines=o3d.utility.Vector2iVector(lines))
        line_set.colors = o3d.utility.Vector3dVector(colors)
        pc_in_chessboard_frame_o3d = o3d_viewer_from_pointcloud(pc_in_chessboard_frame,
                                                                is_normalized=False, is_show=False)
        o3d.visualization.draw_geometries([pc_in_chessboard_frame_o3d, line_set])

    return rmat_lidar_to_chessboard, tvec_in_chessboard_frame, pc_in_chessboard_frame


def fast_polygon_test(pointcloud, bound):
    """
    test whether point in polygon(corners is bound)
    Notice: because the the border of ideal chessboard that we aim to solve is just parallel to x and y axis,
    the polygon test can be simplified extremely
    :param pointcloud:
    :param bound:
    :return:
    """
    not_in_polygon = (pointcloud[:, 0] < min(bound[:, 0])) | (pointcloud[:, 0] > max(bound[:, 0])) \
                     | (pointcloud[:, 1] < min(bound[:, 1])) | (pointcloud[:, 1] > max(bound[:, 1]))
    return not_in_polygon


def visualize_binarization_pointcloud(pointcloud, intensity_threshold):
    binarization_pointcloud = pointcloud.copy()
    intensity = binarization_pointcloud[:, 3].astype(np.int32)

    colors = np.zeros((intensity.shape[0], 3))

    # colors[intensity < intensity_threshold] = np.array([0.0, 0.0, 1.0])
    # colors[intensity >= intensity_threshold] = np.array([1.0, 0.0, 0.0])

    colors[intensity < intensity_threshold] = np.array([0.5, 0.5, 0.5])
    colors[intensity >= intensity_threshold] = np.array([0.9, 0.9, 0.9])

    o3d_viewer_from_pointcloud(binarization_pointcloud, colors=colors)


def load_pointcloud_img_file_pair(dir):
    """
    load the pointcloud and image file pair from the directory
    """
    pointcloud_dir = dir / 'pointcloud'
    img_dir = dir / 'img'
    pointcloud_file_list = sorted(list(pointcloud_dir.glob('*.pcd')))
    img_file_list = sorted(list(img_dir.glob('*.png')))

    assert len(pointcloud_file_list) == len(img_file_list)
    idx_list = list(range(len(pointcloud_file_list)))
    pointcloud_img_file_pair = list(zip(idx_list, pointcloud_file_list, img_file_list))

    return pointcloud_img_file_pair


def update_yaml(file, extrinsic_matrix):
    """
    save the extrinsic parameter
    :param file:
    :param extrinsic_matrix:
    :return:
    """
    with open(file, 'r') as f:
        yaml_config = yaml.load(f, Loader=yaml.FullLoader)
    yaml_config[2]['LIDAR_EXTRINSIC'] = np.ravel(extrinsic_matrix).tolist()

    with open(file, 'w') as f:
        yaml.dump(yaml_config, f, default_flow_style=None)


def matching_loss(theta_t, pointcloud, intensity_pivot, grid_num_along_x, grid_num_along_y, grid_size, corner_color):
    # 先旋转再平移（先转换到模型系下）
    rotation_mat = Rotation.from_rotvec([0, 0, theta_t[0]])
    transform_mat = np.eye(4)
    transform_mat[:3, :3] = rotation_mat.as_matrix()
    transform_mat[:3, 3] = [theta_t[1], theta_t[2], 0]
    pc_hom = np.hstack((pointcloud[:, :3], np.ones((pointcloud.shape[0], 1), dtype=np.float32)))
    trans_pointcloud = pc_hom @ transform_mat.T
    trans_pointcloud[:, 3] = pointcloud[:, 3]

    #  ↑ y
    #  |     bound [4, 2] (x, y) LiDAR frame
    #  -→ x
    bound = np.array(
        [[0, 0],
         [grid_num_along_x, 0],
         [grid_num_along_x, grid_num_along_y],
         [0, grid_num_along_y]]) * grid_size
    bound = bound - np.array([grid_num_along_x, grid_num_along_y]) * grid_size / 2

    bound_x_coord_list = grid_size * \
                         (np.array(range(0, grid_num_along_x + 1)) - float(grid_num_along_x) / 2)
    bound_y_coord_list = grid_size * \
                         (np.array(range(0, grid_num_along_y + 1)) - float(grid_num_along_y) / 2)

    # 从左下角往右上角看，同奇则为标定板底部的颜色
    point_grid = np.int_(1 / grid_size *
                         (trans_pointcloud[:, :2] + [grid_num_along_x * grid_size / 2,
                                                     grid_num_along_y * grid_size / 2]))

    supposed_colors = np.where((point_grid[:, 0] % 2) == (point_grid[:, 1] % 2), 0, 1)
    if corner_color == 'white':
        supposed_colors = 1 - supposed_colors

    actual_colors = (np.sign(trans_pointcloud[:, 3] - intensity_pivot) + 1) / 2

    if 0:
        visualization.o3d_viewer_from_pointcloud(trans_pointcloud, is_normalized=False)
        # 理论情况: 基于行列的奇偶性，相同时则为高反格子
        colors = np.zeros((pointcloud.shape[0], 3))
        colors[supposed_colors == True, :] = [1, 1, 1]
        colors[supposed_colors == False, :] = [0, 0, 0]
        visualization.o3d_viewer_from_pointcloud(trans_pointcloud, colors=colors)
        # 实际情况：基于强度阈值，高反为1（白色），低反为0（黑色）
        colors = np.zeros((pointcloud.shape[0], 3))
        colors[actual_colors == 0, :] = [0, 0, 0]
        colors[actual_colors == 1, :] = [1, 1, 1]
        visualization.o3d_viewer_from_pointcloud(trans_pointcloud, colors=colors)

    # 有两种情况贡献损失值
    # 1. 当激光点不在棋盘格上
    # 2. 当激光点在棋盘格上，但是颜色不匹配
    point_not_in_chessboard = fast_polygon_test(trans_pointcloud, bound)

    point_color_unmatch = (actual_colors != supposed_colors)
    apply_cost_mask = point_color_unmatch | point_not_in_chessboard

    color_mask = point_color_unmatch
    # the cost are calculated by the 1-norm distance between each point and its nearest grid
    # e.g. (100) -> (100, 1) -> (100, 7) -> (100,7) - (7) -> (100,7) - (100,7)
    point_x = (trans_pointcloud[:, 0]).reshape(-1, 1).repeat(len(bound_x_coord_list), axis=1)
    min_dist_in_grid_along_x = np.min(np.abs(point_x - bound_x_coord_list), axis=1)

    point_y = (trans_pointcloud[:, 1]).reshape(-1, 1).repeat(len(bound_y_coord_list), axis=1)
    min_dist_in_grid_along_y = np.min(np.abs(point_y - bound_y_coord_list), axis=1)

    cost_vec = min_dist_in_grid_along_x + min_dist_in_grid_along_y

    cost = np.sum(cost_vec * apply_cost_mask)

    return cost
