import logging
import os
import time

import cv2
import numpy as np
from ampcl.io import load_pointcloud
from ampcl.visualization import (o3d_viewer_from_pointcloud)
import argparse

from common import Config

np.random.seed(233)


class ImageViewBasedFilter(Config):
    def __init__(self, cfg_file):
        super().__init__(cfg_file)

        self.horizon_offset = self.SensorParam.horizon_offset
        self.vertical_offset = self.SensorParam.vertical_offset
        self.horizon_fov = self.SensorParam.horizon_fov
        self.vertical_fov = self.SensorParam.vertical_fov
        self.angle_resolution = self.SensorParam.angle_resolution

        self.img_width = int(self.horizon_fov / self.angle_resolution)
        self.img_height = int(self.vertical_fov / self.angle_resolution)
        self.img_x_resolution = self.img_width / self.horizon_fov
        self.img_y_resolution = self.img_height / self.vertical_fov

    def detect_chessboard(self, intensity_img, mask_img):
        """
        :param:
        :return: True if the region has chessboard
        """
        candidate_in_intensity_img = np.where(mask_img == 255, intensity_img, 0)

        ret, corners = cv2.findChessboardCornersSB(candidate_in_intensity_img, self.chessboard_pattern,
                                                   cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_NORMALIZE_IMAGE)
        corners = np.squeeze(corners)
        if ret and 0:
            debug_img = np.copy(candidate_in_intensity_img)
            debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
            for corner_id in range(len(corners)):
                cv2.circle(debug_img, tuple(corners[corner_id].astype(np.int32)), radius=2,
                           color=(255, 255, 255), thickness=-1, lineType=cv2.LINE_AA)

            description = "visualize the detected chessboard"
            cv2.namedWindow(description, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(description, self.img_width // 2, self.img_height // 2)
            cv2.imshow(description, debug_img)
            cv2.waitKey(0)
            cv2.destroyWindow(description)

        return ret, corners

    @staticmethod
    def xyz_to_yaw_pitch_depth(pointcloud):
        assert len(pointcloud.shape) == 2, "The pointcloud should be [N, ...] e.g. [N, 3]"
        x = pointcloud[:, 0]
        y = pointcloud[:, 1]
        z = pointcloud[:, 2]

        depth = np.linalg.norm(pointcloud[:, :3], axis=1, ord=2)
        yaw = np.arctan2(y, x)
        pitch = np.arcsin(z / depth)

        yaw = np.rad2deg(yaw)
        pitch = np.rad2deg(pitch)
        return yaw, pitch, depth

    @staticmethod
    def yaw_pitch_depth_to_xyz(yaw, pitch, depth):
        yaw = np.deg2rad(yaw)
        pitch = np.deg2rad(pitch)

        x = depth * np.cos(pitch) * np.cos(yaw)
        y = depth * np.cos(pitch) * np.sin(yaw)
        z = depth * np.sin(pitch)

        pointcloud = np.column_stack((x, y, z))
        return pointcloud

    def yaw_pitch_to_img_xy(self, yaw, pitch):
        """
        对激光点的(yaw, pitch)坐标进行量化得到深度图的像素坐标(x, y)
        :param yaw:
        :param pitch:
        :return:
        """

        img_x = (yaw - self.horizon_offset) * self.img_x_resolution
        img_y = self.img_height - (pitch - self.vertical_offset) * self.img_y_resolution

        img_x = np.around(img_x).astype(np.int32)
        img_y = np.around(img_y).astype(np.int32)

        return img_x, img_y

    def img_xy_to_yaw_pitch(self, img_x, img_y):
        """
        对深度图的像素坐标(x, y)进行反量化得到激光点的(yaw, pitch)量化坐标
        :param img_x:
        :param img_y:
        :return:
        """

        yaw = img_x / self.img_x_resolution - self.horizon_offset
        # note: 取反的原因是因为要将竖直方向从下往上坐标递增->从上往下递增（以符合像素坐标系）
        pitch = (self.img_height - img_y) / self.img_y_resolution + self.vertical_offset
        return yaw, pitch

    def img_xy_depth_to_xyz(self, img_x, img_y, depth):
        yaw, pitch = self.img_xy_to_yaw_pitch(img_x, img_y)
        pointcloud = self.yaw_pitch_depth_to_xyz(yaw, pitch, depth)
        return pointcloud

    def reproject(self, point_index, grid_index, extracted_grid_index):
        """
        # todo：画映射图
        从栅格中还原出点云
        :param point_index:
        :param grid_index:
        :param extracted_grid_index:
        :return:
        """

        # 遍历激光点云构建grid_idx和point_idx的映射（一对多）dict<int, vector<int>>
        grid_index_point_index_dict = {}
        for point_idx, grid_idx in zip(point_index, grid_index):
            if grid_idx in grid_index_point_index_dict:
                grid_index_point_index_dict[grid_idx].append(point_idx)
            else:
                grid_index_point_index_dict[grid_idx] = [point_idx]

        extracted_point_index = []
        for extracted_grid_idx in extracted_grid_index:
            if extracted_grid_idx in grid_index_point_index_dict:
                extracted_point_idx = grid_index_point_index_dict[extracted_grid_idx]
                for i in extracted_point_idx:
                    extracted_point_index.append(i)

        return extracted_point_index

    def extract_edges_in_depth_image(self, img, threshold=30):
        """
        相应的卷积核为：
        [1, 0,  1]       [1,   2,  1]
        [2, 0, -2]       [0,   0,  0]
        [1, 0,  1]       [-1, -2, -1]
        :param img:
        :return:
        """
        kernel = np.array([1, 0, -1, 2, 0, -2, 1, 0, -1]).reshape(3, 3)
        img1 = cv2.filter2D(img, -1, kernel)
        img2 = cv2.filter2D(img, -1, -kernel)
        img3 = np.maximum(img1, img2)

        kernel = np.array([1, 2, 1, 0, 0, 0, -1, -2, -1]).reshape(3, 3)
        img4 = cv2.filter2D(img, -1, kernel)
        img5 = cv2.filter2D(img, -1, -kernel)
        img6 = np.maximum(img4, img5)

        edge_img = np.maximum(img3, img6)
        # 通过阈值化去掉不明显的边缘
        ret, edge_img = cv2.threshold(edge_img, threshold, 255, cv2.THRESH_BINARY)

        return edge_img

    def apply(self, pointcloud, debug=False):

        start = time.time()
        # step1: 剔除噪点
        mask = pointcloud[:, 0] >= 0.1
        pointcloud = pointcloud[mask]

        # 计算航向角和俯仰角
        yaw, pitch, depth = self.xyz_to_yaw_pitch_depth(pointcloud)
        img_x, img_y = self.yaw_pitch_to_img_xy(yaw, pitch)

        logging.debug(f"The range of azimuth: ({np.min(yaw)}, {np.max(yaw)})")
        logging.debug(f"The range of vertical view is ({np.min(pitch)}, {np.max(pitch)})")

        # step2：移除超出前视图范围的点和较远的激光点
        mask = (img_x >= 0) & (img_x < self.img_width) \
               & (img_y >= 0) & (img_y < self.img_height) \
               & (depth < 30)

        img_x = img_x[mask]
        img_y = img_y[mask]
        depth = depth[mask]
        pointcloud = pointcloud[mask]

        # step3：依次构建深度图、强度图、哈希表
        raw_depth_img = np.full((self.img_height, self.img_width), 0, dtype=np.uint8)
        raw_intensity_img = np.full((self.img_height, self.img_width), 0, dtype=np.uint8)
        pointcloud_id_img = np.full((self.img_height, self.img_width), -1, dtype=np.int32)

        self.max_depth = np.max(depth)
        raw_depth_img[img_y, img_x] = np.around(depth / self.max_depth * 255).astype(np.uint8)
        raw_intensity_img[img_y, img_x] = (pointcloud[:, 3]).astype(np.uint8)
        pointcloud_id_img[img_y, img_x] = np.arange(pointcloud.shape[0])

        pointcloud_index = np.arange(pointcloud.shape[0])
        grid_index = img_y * self.img_width + img_x

        ret = False
        # step4: 提取强度图下的标定板角点
        for kernel_size in [9, 7, 5, 3]:
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

            # 填充孔洞
            morphology_depth_img = cv2.morphologyEx(raw_depth_img, cv2.MORPH_CLOSE, kernel)

            morphology_intensity_img = cv2.morphologyEx(raw_intensity_img, cv2.MORPH_CLOSE, kernel)
            morphology_intensity_img = np.where(raw_intensity_img == 0, morphology_intensity_img, raw_intensity_img)
            morphology_intensity_img = cv2.medianBlur(morphology_intensity_img, 3)

            ret, corners = cv2.findChessboardCornersSB(morphology_intensity_img, self.chessboard_pattern,
                                                       cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret:
                logging.info(f"Find chessboard corners directly from intensity image with kernel size {kernel_size}")
                # description = "visualize the intensity image after morphology operator"
                # cv2.namedWindow(description, cv2.WINDOW_NORMAL)
                # cv2.resizeWindow(description, morphology_intensity_img.shape[1] // 2,
                #                  morphology_intensity_img.shape[0] // 2)
                # cv2.imshow(description, morphology_intensity_img)
                # cv2.waitKey(0)
                # cv2.destroyWindow(description)
                break

            for threshold in [30, 15, 4]:
                # 生成边缘图
                edge_img = self.extract_edges_in_depth_image(morphology_depth_img, threshold=threshold)

                # 提取轮廓
                contours, hierarchy = cv2.findContours(edge_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

                if self.vis_contour:
                    contour_img_for_debug = np.zeros_like(edge_img)
                    contour_img_for_debug = cv2.cvtColor(contour_img_for_debug, cv2.COLOR_GRAY2BGR)
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        if area < 2000:
                            continue
                        color = (np.random.random(3) * 255).astype(np.uint8)
                        # color = np.array([0, 0, 255], dtype=np.uint8)
                        cv2.drawContours(contour_img_for_debug, [contour], 0, color.tolist(), 2, cv2.LINE_AA)

                    description = "visualize the contour in image"
                    cv2.namedWindow(description, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(description, self.img_width // 2, self.img_height // 2)
                    cv2.imshow(description, contour_img_for_debug)
                    cv2.waitKey(0)
                    cv2.destroyWindow(description)

                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area < 2000:
                        continue

                    # 构建区域掩模
                    contour_mask = np.zeros_like(edge_img)
                    cv2.drawContours(contour_mask, [contour], 0, 255, thickness=-1)

                    ret, corners = self.detect_chessboard(morphology_intensity_img, contour_mask)

                    if ret:
                        break

                if ret:
                    logging.debug(f"Find chessboard corners with kernel size {kernel_size} and threshold {threshold}")
                    break

            if ret:
                break
        logging.debug(f'[Filter] the front view-based filter takes: {1000 * (time.time() - start):.2f} ms')

        if not ret:
            logging.warning("[Segmentation] No chessboard can be detected in front view representation")
            return
        else:
            # step5: 提取点云
            grid_size_in_pixel = np.linalg.norm(corners[0] - corners[1])

            hull = cv2.convexHull(corners.astype(np.int32))
            rect = list(cv2.minAreaRect(hull))
            rect[0] = list(rect[0])
            rect[1] = list(rect[1])
            scale = self.boundary_threshold
            rect[1][0] += scale * grid_size_in_pixel
            rect[1][1] += scale * grid_size_in_pixel

            contour_point = cv2.boxPoints(rect).astype(np.int32)
            mask_background = np.zeros_like(morphology_intensity_img)
            cv2.drawContours(mask_background, [contour_point], 0, 255, -1, lineType=cv2.LINE_AA)

            pointcloud_id_region = pointcloud_id_img[mask_background > 0]
            pointcloud_id = pointcloud_id_region[pointcloud_id_region > 0]

            # 提取区域大于0的位置而不是里面的像素值
            if self.use_hash_map_reproject or pointcloud_id.shape[0] < 15000:
                mask_pointcloud_id_img = np.where(mask_background > 0, pointcloud_id_img, 0)
                extracted_grid_index = np.flatnonzero(mask_pointcloud_id_img)
                extracted_pc_index = self.reproject(pointcloud_index, grid_index, extracted_grid_index)
                chessboard = pointcloud[extracted_pc_index]
            else:
                chessboard = pointcloud[pointcloud_id]

            candidate_in_depth_img = np.where(mask_background > 0, morphology_depth_img, 0)

            # note: 论文中没提及这一步，但无伤大雅，后续的RANSAC能起到相同的作用。
            depth_histogram = np.bincount(np.ravel(candidate_in_depth_img[candidate_in_depth_img > 1e-6]))
            depth_with_max_count = np.argmax(depth_histogram)
            chessboard_depth = depth_with_max_count / 255 * self.max_depth
            mask = np.abs(np.linalg.norm(chessboard[:, :3], axis=1) - chessboard_depth) < 0.5
            chessboard = chessboard[mask]

            if 0:
                description = "visualize the chessboard boundary in image"
                debug_img = cv2.cvtColor(morphology_intensity_img, cv2.COLOR_GRAY2BGR)

                cv2.namedWindow(description, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(description, self.img_width // 2, self.img_height // 2)
                cv2.circle(debug_img, center=np.int32(geometric_center), radius=1, color=(0, 0, 255), thickness=1,
                           lineType=cv2.LINE_AA)
                cv2.drawContours(debug_img, [contour_point], 0, (255, 0, 0), 1, cv2.LINE_AA)
                cv2.imshow(description, debug_img)
                cv2.waitKey(0)
                cv2.destroyWindow(description)

            if self.vis_raw_chessboard_pc:
                o3d_viewer_from_pointcloud(chessboard, is_normalized=False)

            return chessboard


def target_segmentatino_demo(config_file, idx=None):
    filter = ImageViewBasedFilter(config_file)
    dataset_dir = filter.dataset_dir
    data_dir = f"{dataset_dir}/pointcloud"
    pc_list = sorted(os.listdir(data_dir))

    if idx is not None:
        pc_list = [pc_list[i] for i in idx]

    for pc_file in pc_list:
        pc_file = f"{data_dir}/{pc_file}"
        logging.info(f"[IO] current file is {pc_file}")
        pointcloud = load_pointcloud(pc_file)
        filter = ImageViewBasedFilter(config_file)
        filter.apply(pointcloud, debug=False)


def simple_demo(pc_file, config_file):
    pointcloud = load_pointcloud(pc_file)
    filter = ImageViewBasedFilter(config_file)
    filter.apply(pointcloud, debug=False)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG, format='[%(levelname)s] %(message)s')
    parser = argparse.ArgumentParser()
    parser.add_argument('--idx', nargs='+', type=int, default=[2])
    parser.add_argument('--cfg', type=str, default="config/horizon.yaml")
    args = parser.parse_args()
    cfg = args.cfg
    target_segmentatino_demo("config/horizon.yaml", idx=args.idx)
