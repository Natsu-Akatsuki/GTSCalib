import argparse
import logging
import os
import pickle
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pandas
import yaml
from ampcl import filter
from ampcl.calibration import calibration
from ampcl.io import pointcloud_io
from colorama import Fore, Style
from easydict import EasyDict
from matplotlib import pyplot as plt

from common import load_pointcloud_img_file_pair
from ampcl import calibration


class Experiment():
    def __init__(self, config_file=None, calib_file=None):

        if config_file is not None:
            with open(config_file, 'r') as f:
                yaml_config = yaml.load(f, Loader=yaml.FullLoader)
                dot_config = EasyDict(yaml_config)
            dataset_dir = dot_config.Dataset.DataDir
            self.dataset_dir = str(Path(dataset_dir).resolve())
            calib_file = os.path.join(self.dataset_dir, "sensor.yaml")
            self.corners_file = os.path.join(self.dataset_dir, "corners.pkl")

        if not isinstance(calib_file, dict):
            calib = self.get_calib_from_file(calib_file)
        else:
            calib = calib_file

        self.intri_matrix = calib['intri_matrix']
        self.distor = calib['distor']
        self.extri_matrix = calib['extri_matrix']

        self.cal_info = {
            "intri_mat": self.intri_matrix,
            "distor": self.distor,
            "extri_mat": self.extri_matrix
        }

    @staticmethod
    def get_calib_from_file(file):
        with open(file, 'r') as f:
            yaml_config = yaml.load(f, Loader=yaml.FullLoader)

        intri_matrix = np.array(yaml_config[0]['CAMERA_INTRINSIC']).reshape(3, 3)
        distor = np.array(yaml_config[1]['CAMERA_DISTORTION']).reshape(5)

        if yaml_config[2]['LIDAR_EXTRINSIC'][0] is not None:
            extri_matrix = np.array(yaml_config[2]['LIDAR_EXTRINSIC']).reshape(4, 4)
        else:
            extri_matrix = None

        data = {"intri_matrix": intri_matrix,
                "distor": distor,
                "extri_matrix": extri_matrix}

        return data

    def project_pointcloud_to_img(self, pointcloud, img, debug=False):

        # un-distort the image
        raw_img = img.copy()
        img = cv2.undistort(src=img, cameraMatrix=self.intri_matrix, distCoeffs=self.distor)

        # project the pointcloud to img
        pts_img, _ = self.lidar_to_img(pointcloud[:, :3])

        # round
        pts_img = np.floor(pts_img)
        mask = (pts_img[:, 0] > 0) & (pts_img[:, 0] < img.shape[1]) & \
               (pts_img[:, 1] > 0) & (pts_img[:, 1] < img.shape[0])
        pts_img = pts_img[mask]

        # append pseudo color based on intensity
        intensity = pointcloud[:, 3][mask]
        normalized_min = np.min(intensity)
        normalized_max = np.max(intensity)

        # note: [:,:3]: remove the alpha channel
        colors = plt.get_cmap('jet')((intensity - normalized_min) / (normalized_max - normalized_min))[:, :3] * 255
        colors[...] = colors[:, ::-1]  # RGB to BGR

        # achieve the pseudo color
        img[np.int_(pts_img[:, 1]), np.int_(pts_img[:, 0])] = colors

        # visualize
        if debug:
            debug_img = np.hstack((raw_img, img))
            description = "Projection"
            cv2.namedWindow(description, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(description, debug_img.shape[1] // 4, debug_img.shape[0] // 2)
            cv2.imshow(description, debug_img)
            cv2.waitKey(0)
            cv2.destroyWindow(description)

    def project_3d_corners_to_img(self, corners_3d, corners_2d, img):

        tvec = self.extri_matrix[:3, 3].reshape(-1, 1)
        rvec, _ = cv2.Rodrigues(self.extri_matrix[:3, :3])

        pts_img = (cv2.projectPoints(corners_3d[:, :3].astype(np.float32),
                                     rvec, tvec,
                                     self.intri_matrix, self.distor))[0].squeeze()

        colors = np.expand_dims([0, 0, 255], axis=0).repeat(corners_3d.shape[0], axis=0).astype(np.uint8)
        # colors[0] = [255, 0, 0]
        # colors[1] = [0, 255, 0]
        colors[...] = colors[:, ::-1]  # RGB to BGR

        # 可视化 2D 的投影点云（蓝色）
        for (x, y), c in zip(pts_img, colors):
            x, y = np.int_(x), np.int_(y)
            # 图片，圆心位置位置，圆半径，圆颜色，边界厚度（-1：填充）
            cv2.circle(img, (x, y), 2, c.tolist(), 1, lineType=cv2.LINE_AA)

        for corner_id in range(len(corners_2d)):
            cv2.circle(img, tuple(corners_2d[corner_id].astype(np.int32)), radius=1,
                       color=(255, 255, 255), thickness=0, lineType=cv2.LINE_AA)
            # cv2.putText(img, f'{corner_id}', tuple(corners_2d[corner_id].astype(np.int32) + 4),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, lineType=cv2.LINE_AA)

        debug_img = img
        description = "Projection"
        cv2.namedWindow(description, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(description, debug_img.shape[1] // 2, debug_img.shape[0] // 2)
        cv2.imshow(description, debug_img)
        cv2.waitKey(0)
        cv2.destroyWindow(description)

    def evaluate_for_realitiy(self, data, extri_matrix=None, log=True, do_sort=True, do_sim=False):

        if isinstance(data, str):
            with open(data, 'rb') as f:
                corners_in_lidar_list, corners_in_image_list = pickle.load(f)
        if isinstance(data, dict):
            corners_in_lidar_list, corners_in_image_list = data['corners_in_lidar'], data['corners_in_image']

        tvec = extri_matrix[:3, 3].reshape(-1, 1)
        rvec, _ = cv2.Rodrigues(extri_matrix[:3, :3])

        mse_per_placement_list = []
        rmse_per_placement_list = []
        chessboard_to_camera_dist_list = []
        chessboard_to_lidar_dist_list = []
        chessboard_x_in_camera_list = []
        chessboard_y_in_camera_list = []
        chessboard_z_in_camera_list = []
        idx_list = []

        for idx, (corners_in_lidar, corners_in_image) in enumerate(zip(corners_in_lidar_list, corners_in_image_list)):
            # note: 图像的角点坐标是超分辨率的
            corners_in_lidar = corners_in_lidar[:, :3].astype(np.float32)
            projected_corners = (cv2.projectPoints(corners_in_lidar[:, :3],
                                                   rvec, tvec,
                                                   self.intri_matrix, self.distor))[0].squeeze()

            l2_error = np.linalg.norm(projected_corners - corners_in_image, axis=1, ord=2)

            # 标定板距离相机原点的距离
            pts_camera = self.lidar_to_camera_points(corners_in_lidar[:, :3])

            distance_chessboard_to_camera = np.mean(
                np.linalg.norm(pts_camera[:, :3], axis=1, ord=2).astype(np.float32))
            distance_chessboard_to_lidar = np.mean(
                np.linalg.norm(corners_in_lidar[:, :3], axis=1, ord=2).astype(np.float32))

            # 单帧图片的 MSE 和 RMSE
            mse = np.mean(l2_error ** 2)
            mse_per_placement_list.append(mse)
            rmse = np.sqrt(mse)
            rmse_per_placement_list.append(rmse)

            idx_list.append(idx)
            chessboard_to_camera_dist_list.append(distance_chessboard_to_camera)
            chessboard_to_lidar_dist_list.append(distance_chessboard_to_lidar)
            chessboard_x_in_camera_list.append(np.mean(pts_camera[:, 0]))
            chessboard_y_in_camera_list.append(np.mean(pts_camera[:, 1]))
            chessboard_z_in_camera_list.append(np.mean(pts_camera[:, 2]))

        if do_sort:
            # 根据标定板离相机系原点的距离进行排序
            sort_idx = np.argsort(chessboard_to_lidar_dist_list)
            idx_list = np.array(idx_list)[sort_idx]
            chessboard_to_camera_dist_list = np.asarray(chessboard_to_camera_dist_list)[sort_idx]
            chessboard_to_lidar_dist_list = np.asarray(chessboard_to_lidar_dist_list)[sort_idx]
            chessboard_x_in_camera_list = np.asarray(chessboard_x_in_camera_list)[sort_idx]
            chessboard_y_in_camera_list = np.asarray(chessboard_y_in_camera_list)[sort_idx]
            chessboard_z_in_camera_list = np.asarray(chessboard_z_in_camera_list)[sort_idx]
            rmse_per_placement_list = np.asarray(rmse_per_placement_list)[sort_idx]

        if log:
            result = {'idx': idx_list,
                      'LDistance': chessboard_to_lidar_dist_list,
                      'CDistance': chessboard_to_camera_dist_list,
                      'Cx': chessboard_x_in_camera_list,
                      'Cy': chessboard_y_in_camera_list,
                      'Cz': chessboard_z_in_camera_list,
                      'RMSE': rmse_per_placement_list}
            data_frame = pandas.DataFrame(data=result)
            data_frame = data_frame.round(2)
            pandas.set_option('display.colheader_justify', 'center')
            data_frame.to_csv('experiment.csv')
            with pandas.option_context('display.max_rows', None,
                                       'display.max_columns', None,
                                       'display.precision', 3,
                                       ):
                logging.info(f"\n{data_frame}")

        return np.sqrt(np.mean(mse_per_placement_list))

    def qualitative_experiment(self, idx=None):
        pointcloud_img_file_pair = load_pointcloud_img_file_pair(Path(self.dataset_dir))
        if idx is not None:
            pointcloud_img_file_pair = [pointcloud_img_file_pair[i] for i in idx]

        if Path(self.corners_file).exists():
            with open(self.corners_file, 'rb') as f:
                corners_in_lidar_list, corners_in_image_list = pickle.load(f)

        for i, pointcloud_file, img_file in pointcloud_img_file_pair:
            pc_np = pointcloud_io.load_pointcloud(str(pointcloud_file))
            img = cv2.imread(str(img_file))

            self.project_pointcloud_to_img(pc_np, img, debug=True)
            calibration.paint_pointcloud(pc_np, img, self.cal_info, debug=True)


def cli():
    parser = argparse.ArgumentParser(description="arg parser")
    parser.add_argument('--cfg', type=str, default='config/config.yaml',
                        help='specify the calibration config for experiment')
    parser.add_argument('--do_qual', action='store_true', default=True,
                        help='whether to perform qualitative experiment')
    parser.add_argument('--do_quan', action='store_true', default=True,
                        help='whether to perform quantitative experiment')
    parser.add_argument('--idx', nargs='+', type=int, default=[3],
                        help='specify the n-th pairs for quantitative experiment')
    args = parser.parse_args()

    experiment = Experiment(config_file=args.cfg)

    # perform quantitative experiment
    if args.do_quan:
        RRMSE = experiment.evaluate_for_realitiy(data=experiment.corners_file,
                                                 extri_matrix=experiment.extri_matrix,
                                                 log=False)

        logging.info(f'{Fore.BLUE}[Reality] The RRMSE: is {Fore.RED}{RRMSE:.4f}{Fore.BLUE} pixel')

    # perform qualitative experiment
    if args.do_qual:
        if isinstance(args.idx, int):
            args.idx = [args.idx]
        experiment.qualitative_experiment(args.idx)


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    cli()
