import logging
import pickle
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from ampcl import filter, visualization
from ampcl.io import load_pointcloud, save_pointcloud
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from skspatial.objects import Plane, Points
import argparse
import common as pcl
import target_segmentation
from common import (Config, load_pointcloud_img_file_pair, matching_loss,
                    update_yaml)
from experiment import Experiment


def filter_sample_idx_list(data, sample_idx_list=None):
    if sample_idx_list is not None:
        corners_in_lidar_sample_list = [data["corners_in_lidar"][i] for i in sample_idx_list]
        corners_in_image_sample_list = [data["corners_in_image"][i] for i in sample_idx_list]
    else:
        corners_in_lidar_sample_list = data["corners_in_lidar"]
        corners_in_image_sample_list = data["corners_in_image"]
    return corners_in_image_sample_list, corners_in_lidar_sample_list


class CalibrationNode(Config):
    def __init__(self, cfg_file="config/calibration.yaml"):
        super().__init__(cfg_file)

        # for experiment
        calib_file = f"{self.dataset_dir}/sensor.yaml"
        calib = Experiment.get_calib_from_file(calib_file)
        self.intri_matrix = calib['intri_matrix']
        self.distor = calib['distor']
        self.extri_matrix = calib['extri_matrix']
        self.acsc_experiment = Experiment(calib_file=calib_file)

        self.chessboard_pointcloud_dir = Path(f"{self.dataset_dir}/chessboard_pointcloud")
        self.chessboard_pointcloud_dir.mkdir(parents=True, exist_ok=True)

        self.chessboard_filter = target_segmentation.ImageViewBasedFilter(cfg_file)
        self.i = 1

    def segment_chessboard_pointcloud(self, raw_chessboard_pointcloud):
        """
        :param raw_chessboard_pointcloud:
        :return:
        """

        intensity = raw_chessboard_pointcloud[:, 3]
        raw_chessboard_pointcloud = raw_chessboard_pointcloud[intensity > 0]

        # 剔除离群点和使用最小二乘法实现平面拟合
        downsample_chessboard_pointcloud = filter.c_voxel_filter(raw_chessboard_pointcloud,
                                                                 voxel_size=[0.01, 0.01, 0.01],
                                                                 mode=self.voxel_grid_method, log=True)
        plane_model, downsample_inliers = filter.c_ransac_plane_fitting(downsample_chessboard_pointcloud,
                                                                        distance_threshold=self.plane_fitting_threshold,
                                                                        use_optimize_coeff=False,
                                                                        random=False,
                                                                        log=True,
                                                                        max_iterations=100,
                                                                        debug=self.vis_plane_fitting_result)

        # 计算强度的分割阈值
        intensity_threshold = pcl.find_intensity_threshold(downsample_inliers[:, 3],
                                                           methods=self.binarization_methods,
                                                           debug=self.vis_intensity_threshold)

        if self.use_plane_refine:
            plane_model = self.refine_plane(downsample_inliers)
            intensity = downsample_inliers[:, 3]

        # 正交投影
        proj_chessboard_pointcloud = pcl.ray_projection(plane_model,
                                                        downsample_inliers,
                                                        debug=self.vis_proj_chessboard_pc)

        if self.vis_binary_chessboard_pc:
            pcl.visualize_binarization_pointcloud(proj_chessboard_pointcloud, intensity_threshold)

        # 获取标定板点云的坐标系
        rmat_lidar_to_chessboard, tvec_in_chessboard_frame, chessboard_pc_in_chessboard_frame = pcl.pca(
            proj_chessboard_pointcloud, debug=self.vis_pca_pc,
            is_bottom_side_long=self.is_bottom_side_long)

        if self.vis_pca_pc:
            center = np.mean(proj_chessboard_pointcloud[:, :3], axis=0)
            coordinate_point = np.array(
                [center, center + rmat_lidar_to_chessboard[0],
                 center + rmat_lidar_to_chessboard[1],
                 center + rmat_lidar_to_chessboard[2]])
            lines = [[0, 1], [0, 2], [0, 3]]
            colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(coordinate_point),
                                            lines=o3d.utility.Vector2iVector(lines))
            line_set.colors = o3d.utility.Vector3dVector(colors)
            pointcloud_o3d = o3d.geometry.PointCloud()
            pointcloud_o3d.points = o3d.utility.Vector3dVector(raw_chessboard_pointcloud[:, 0:3])
            colors = visualization.intensity_to_color_o3d(raw_chessboard_pointcloud[:, 3], is_normalized=False)
            pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors)
            o3d.visualization.draw_geometries([pointcloud_o3d, line_set])

        return raw_chessboard_pointcloud, chessboard_pc_in_chessboard_frame, tvec_in_chessboard_frame, rmat_lidar_to_chessboard, intensity_threshold

    def refine_plane(self, chessboard_pointcloud):
        plane_point = Points(chessboard_pointcloud[:, :3])
        plane = Plane.best_fit(plane_point)
        n = plane.normal
        plane_model = np.zeros(4)
        plane_model[:3] = n
        plane_model[3] = -plane.point[0] * n[0] - plane.point[1] * n[1] - plane.point[2] * n[2]
        return plane_model

    def extract_correspondences(self, idx=None):

        pointcloud_img_file_pair = load_pointcloud_img_file_pair(Path(self.dataset_dir))
        corners_in_lidar_list = []
        corners_in_image_list = []

        if idx is not None:
            pointcloud_img_file_pair = [pointcloud_img_file_pair[i] for i in idx]

        for idx, pointcloud_file, img_file in pointcloud_img_file_pair:
            pointcloud = load_pointcloud(str(pointcloud_file))
            img = cv2.imread(str(img_file))

            # step1: 从激光点云中提取标定板点云
            chessboard_pc_file = self.chessboard_pointcloud_dir / pointcloud_file.name
            if not self.reset_segmentation and chessboard_pc_file.exists():
                raw_chessboard_pointcloud = load_pointcloud(str(chessboard_pc_file))
            elif self.reset_segmentation or not chessboard_pc_file.exists():
                raw_chessboard_pointcloud = self.chessboard_filter.apply(pointcloud, debug=self.vis_raw_chessboard_pc)
                if raw_chessboard_pointcloud is None or raw_chessboard_pointcloud.shape[0] == 0:
                    continue
                save_pointcloud(raw_chessboard_pointcloud, str(chessboard_pc_file))

            chessboard_pointcloud, pc_in_chessboard_frame, trans_3d, rot_3d, intensity_pivot = \
                self.segment_chessboard_pointcloud(raw_chessboard_pointcloud)

            corners_in_lidar, final_cost = self.detect_3d_corners(pointcloud, chessboard_pointcloud,
                                                                  pc_in_chessboard_frame, trans_3d, rot_3d,
                                                                  intensity_pivot)

            corners_in_image = self.detect_2d_corners(img)
            logging.info(f'{pointcloud_file.name}: loss: {final_cost}')

            if final_cost > 100:
                logging.warning("Can not detect 3D corners, skip this pair")
                continue
            if corners_in_image is None:
                logging.warning("Can not detect 2D corners, skip this pair")
                continue

            corners_in_lidar_list.append(corners_in_lidar)
            corners_in_image_list.append(corners_in_image)

        logging.warning(f"skip {len(pointcloud_img_file_pair) - len(corners_in_lidar_list)} pairs")
        file_name = f"{self.dataset_dir}/corners.pkl"
        with open(file_name, 'wb') as f:
            pickle.dump((corners_in_lidar_list, corners_in_image_list), f)

    def solve_extrinsic_param(self, corners_3d_list, corners_2d_list, do_save=True):

        corners_3d = np.row_stack(corners_3d_list)[:, :3]
        corners_2d = np.row_stack(corners_2d_list)[:, :2]

        assert len(corners_3d) == len(corners_2d), 'The numbers of corners should be equal'

        methods = self.pnp_method
        methods.append("RANSAC")

        ret_list = []
        rvec_list = []
        tvec_list = []
        reproj_error_list = []
        for method in methods:
            corners_3d = corners_3d.astype(np.float32)
            corners_2d = corners_2d.astype(np.float32)
            if method != "RANSAC":
                cv_method = eval(f"cv2.SOLVEPNP_{method}")
                ret, rvec, tvec, reproj_error = cv2.solvePnPGeneric(corners_3d,
                                                                    corners_2d,
                                                                    self.intri_matrix,
                                                                    self.distor,
                                                                    flags=cv_method)
            else:
                ret, rvec, tvec, inliers = cv2.solvePnPRansac(corners_3d.astype(np.float32), corners_2d,
                                                              self.intri_matrix, self.distor)
                ransac_reproj_error = cv2.norm(
                    corners_2d - cv2.projectPoints(corners_3d, rvec, tvec, self.intri_matrix, self.distor)[0],
                    cv2.NORM_L2) / len(corners_2d)
                if corners_3d.shape[0] != inliers.shape[0]:
                    print(f"there is outliers")
                if ransac_reproj_error > 2000:
                    logging.warning(f"RANSAC reproj_error is too large: {ransac_reproj_error}")
                    logging.warning("please check: \n"
                                    "1. whether the intrinsic parameter is accurate\n"
                                    "2. whether there are two frames are coplaner or too close")
                break

            ret_list.append(ret)
            rvec_list.append(rvec[0])
            tvec_list.append(tvec[0])
            reproj_error_list.append(reproj_error[0])

        method_id = np.argmin(reproj_error_list)
        rvector = rvec_list[method_id]
        tvector = tvec_list[method_id]
        logging.debug(f"using solvePnPGeneric with {methods[method_id]}")

        rotation_mat, _ = cv2.Rodrigues(rvector)
        extrinsic_matrix = np.hstack([rotation_mat, tvector])
        extrinsic_matrix = np.vstack([extrinsic_matrix, [0, 0, 0, 1]])

        if do_save:
            update_yaml(f"{str(self.dataset_dir)}/sensor.yaml", extrinsic_matrix)

        return extrinsic_matrix

    def detect_2d_corners(self, img):
        """
        detect corners in camera image
        :param img:
        :param debug:
        :return:
        """

        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(gray_img, self.chessboard_pattern,
                                                   cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)

        if ret:
            # remove unused dimension
            corners = np.squeeze(corners)

            # 确保第一个角点的顺序位于图像的左下角
            #             (x1,y1)
            #
            # (x0,y0)
            # x0 < x1; y0 > y1
            if corners[0, 1] < corners[-1, 1]:
                # make sure the first corner is under the last corner
                corners = corners[::-1, :]
            elif corners[0, 1] == corners[-1, 1]:
                # make sure the first corner is on the left of the last corner
                if corners[0, 0] > corners[-1, 0]:
                    corners = corners[::-1, :]

            if self.vis_2D_corners:
                # 查看角点检测效果
                debug_img = np.copy(img)
                debug_img = cv2.drawChessboardCorners(debug_img, self.chessboard_pattern, corners, ret)
                for corner_id in range(len(corners)):
                    cv2.circle(debug_img, tuple(corners[corner_id].astype(np.int32)), radius=1,
                               color=(0, 0, 255), thickness=-1)
                    cv2.putText(debug_img, f'{corner_id}', tuple(corners[corner_id].astype(np.int32) + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.2, (255, 0, 255), 1)

                cv2.namedWindow("debug", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("debug", debug_img.shape[1] // 4, debug_img.shape[0] // 4)
                cv2.imshow("debug", debug_img)
                cv2.waitKey(0)
            return corners
        else:
            return None

    def detect_3d_corners(self, pointcloud,
                          chessboard_pointcloud,
                          pc_in_chessboard_frame,
                          trans_3d, rot_3d,
                          intensity_pivot):

        # 基于实际标定板的坐标系，构建理想标定板
        ideal_chessboard_corner = self.generate_ideal_model_chessboard_corners(
            debug=self.vis_ideal_model_chessboard_corners)

        # ILCC: Powell | ACSC: L-BFGS-B
        rst = minimize(matching_loss,
                       x0=np.zeros(3),
                       args=(
                           pc_in_chessboard_frame.copy(), intensity_pivot, self.corner_num_along_bottom_side + 1,
                           self.corner_num_along_left_side + 1,
                           self.grid_size,
                           self.corner_color),
                       method='L-BFGS-B', tol=1e-10,
                       options={"maxiter": 10000})

        point_size = pc_in_chessboard_frame.shape[0]
        final_cost = rst.fun / point_size * 1e4

        theta, offset_x, offset_y = rst.x

        if theta * 57.3 > 90:
            print("ERROR: theta > 90")

        rot_2d = Rotation.from_rotvec([0, 0, theta]).as_matrix()
        trans_2d = np.array([offset_x, offset_y, 0])

        # 获取棋盘格角点在激光雷达坐标系下的坐标
        chessboard_corner_in_lidar = self.chessboard_to_lidar_transform(ideal_chessboard_corner.copy(), rot_2d,
                                                                        trans_2d,
                                                                        rot_3d, trans_3d)

        if self.vis_3D_corners_in_lidar_frame:
            # 将corner还原到原始点云对应的3D空间
            pc_in_lidar_frame = (pc_in_chessboard_frame[:, :3] + trans_3d) @ rot_3d

            # 测试棋盘格角点的间隔
            # np.linalg.norm(chessboard_corner_in_lidar[0, :3] - chessboard_corner_in_lidar[1, :3], ord=2)

            # 可视化角点 (visualize the chessboard and its corners)
            o3d_object = visualization.visualize_point_with_sphere(chessboard_corner_in_lidar, radius=0.01,
                                                                   color=(0, 0, 0))
            # o3d_object[0].paint_uniform_color([1, 0, 0])
            # o3d_object[1].paint_uniform_color([0, 1, 0])

            # 可视化棋盘格点云
            chessboard_pc_o3d = o3d.geometry.PointCloud()
            chessboard_pc_o3d.points = o3d.utility.Vector3dVector(chessboard_pointcloud[:, 0:3])
            intensity_threshold = intensity_pivot
            intensity = chessboard_pointcloud[:, 3]
            colors = visualization.intensity_to_color_o3d(intensity, is_normalized=False)
            # colors = np.zeros((intensity.shape[0], 3))
            # colors[intensity < intensity_threshold] = np.array([0.5, 0.5, 0.5])
            # colors[intensity >= intensity_threshold] = np.array([0.9, 0.9, 0.9])
            chessboard_pc_o3d.colors = o3d.utility.Vector3dVector(colors)
            o3d_object.append(chessboard_pc_o3d)

            # 显示点云
            # pointcloud_o3d = o3d.geometry.PointCloud()
            # pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud[:, 0:3])
            # colors = visualization.intensity_to_color_o3d(pointcloud[:, 3], is_normalized=False)
            # pointcloud_o3d.colors = o3d.utility.Vector3dVector(colors)
            # o3d_object.append(pointcloud_o3d)

            # 显示坐标系
            # coordinate_point = np.array(
            #     [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
            # lines = [[0, 1], [0, 2], [0, 3]]
            # colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            # frame_o3d = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(coordinate_point),
            #                                  lines=o3d.utility.Vector2iVector(lines))
            # frame_o3d.colors = o3d.utility.Vector3dVector(colors)
            # o3d_object.append(frame_o3d)

            o3d.visualization.draw_geometries(o3d_object)

        return chessboard_corner_in_lidar, final_cost

    @staticmethod
    def chessboard_to_lidar_transform(corners, rot_2d, trans_2d, rot_3d, trans_3d):
        # transform corners from ideal-model frame to pca xoy plane-model frame
        # 坐标变换：chessboard model frame -> chessboard frame
        corners[:, :3] = corners[:, :3] @ rot_2d - trans_2d
        # 坐标变换：chessboard frame -> lidar frame
        corners[:, :3] = (corners[:, :3] + trans_3d) @ rot_3d

        # make sure the index start from the bottom left to preserve the same corners' sequence as img
        if corners[0, 2] > corners[-1, 2]:  # compare the z axis
            # make sure the first corner is under the last corner
            corners = corners[::-1, :]
        elif corners[0, 2] == corners[-1, 2]:
            # make sure the first corner is on the left of the last corner
            if corners[0, 1] < corners[-1, 1]:  # compare the y axis
                corners = corners[::-1, :]

        return corners

    def generate_ideal_model_chessboard_corners(self, debug=False):
        grid_num_along_left_side = self.corner_num_along_left_side + 1
        grid_num_along_bottom_side = self.corner_num_along_bottom_side + 1
        border_size_along_bottom_side = np.array(grid_num_along_bottom_side * self.grid_size)
        border_size_along_left_side = np.array(grid_num_along_left_side * self.grid_size)

        xx, yy = np.meshgrid(np.arange(1, grid_num_along_bottom_side) * self.grid_size,
                             np.arange(1, grid_num_along_left_side) * self.grid_size)
        xx, yy = xx.reshape(-1), yy.reshape(-1)
        zz = np.zeros_like(yy)
        corners = np.column_stack((xx, yy, zz))

        # 6 13 ... 34
        # .        .
        # .        .
        # .        .
        # 0 7  ... 28
        #
        corner_index = (yy / self.grid_size - 1) + self.corner_num_along_left_side * (xx / self.grid_size - 1)
        corners = np.column_stack([corners, corner_index])

        # add center offset
        corners[:, :2] -= (border_size_along_bottom_side / 2, border_size_along_left_side / 2)
        corners = corners[np.argsort(corner_index), :]

        if debug:

            fig, ax = plt.subplots()
            for i in range(len(corners)):
                plt.text(corners[i, 0] * 100, corners[i, 1] * 100, f'{np.int_(corners[i, 3])}', color='r')

            plt.scatter(corners[:, 0] * 100, corners[:, 1] * 100, s=5, color="red")
            plt.xlim(-border_size_along_bottom_side * 100 / 2, border_size_along_bottom_side * 100 / 2)
            plt.xlabel('x (cm)')
            x_ticks = np.arange(-border_size_along_bottom_side * 100 / 2, border_size_along_bottom_side * 100 / 2, 5)
            plt.xticks(x_ticks)
            ax.axis('equal')
            plt.show()

        return corners

    def estimate(self, data, estimate_sample_idx_list=None, do_save=False):
        """
        :param do_save:
        :param data:
        :param estimate_sample_idx_list: None means estimate all the data
        """
        logging.info(f"\nEstimate sample idx : {estimate_sample_idx_list}")

        corners_in_image_sample_list, corners_in_lidar_sample_list = filter_sample_idx_list(data,
                                                                                            estimate_sample_idx_list)

        return self.solve_extrinsic_param(corners_in_lidar_sample_list, corners_in_image_sample_list, do_save=do_save)

    def evaluate(self, data, extrinsic_mat, evaluate_sample_idx_list=None):

        corners_in_image_sample_list, corners_in_lidar_sample_list = filter_sample_idx_list(data,
                                                                                            evaluate_sample_idx_list)

        selected_data = {
            "corners_in_lidar": corners_in_lidar_sample_list,
            "corners_in_image": corners_in_image_sample_list
        }

        result = self.acsc_experiment.evaluate_for_realitiy(selected_data, extrinsic_mat, do_sort=True, log=True)
        logging.info(f"{evaluate_sample_idx_list}: Mean Square Error = {np.mean(result):.4f}")

    def load_corners_from_file(self, corners_file=None):
        if corners_file is None:
            corners_file = f"{self.dataset_dir}/corners.pkl"
        else:
            corners_file = corners_file

        with open(corners_file, 'rb') as f:
            corners_in_lidar_list, corners_in_image_list = pickle.load(f)

        corners_data = {
            "corners_in_lidar": corners_in_lidar_list,
            "corners_in_image": corners_in_image_list
        }

        return corners_data


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default="config/config.yaml")

    args = parser.parse_args()
    calibration_node = CalibrationNode(cfg_file=args.cfg)

    sample_idx_list_for_extraction = None
    calibration_node.extract_correspondences(idx=sample_idx_list_for_extraction)

    estimate_sample_idx_list = None
    evaluate_sample_idx_list = None
    data = calibration_node.load_corners_from_file()
    if len(data['corners_in_lidar']) == 0:
        logging.error("No corners data, please extract corners first")
        exit(1)
    extri_mat = calibration_node.estimate(data, estimate_sample_idx_list=estimate_sample_idx_list, do_save=True)
