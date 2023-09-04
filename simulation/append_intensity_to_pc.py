from pathlib import Path

import cv2
import numpy as np
import yaml
from ampcl import io
from ampcl import visualization
from ampcl.calibration import Calibration
from tqdm import tqdm

from common import load_pointcloud_img_file_pair


class IntensityRestoration(Calibration):
    def __init__(self, calib_file=None):

        if not isinstance(calib_file, dict):
            calib = self.get_calib_from_file(calib_file)
        else:
            calib = calib_file

        self.intri_matrix = calib['intri_matrix']
        self.distor = calib['distor']
        self.extri_matrix = calib['extri_matrix']

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

    def append_intensity_to_pc(self, pointcloud, img, debug=False):

        # passthrough filter
        mask = (pointcloud[:, 0] >= 0.1) & (pointcloud[:, 0] <= 120.0)
        pointcloud = pointcloud[mask]

        # project the point onto image
        proj_img, depth_camera = self.lidar_to_img(pointcloud[:, :3])

        # un-distort the image
        img = cv2.undistort(src=img, cameraMatrix=self.intri_matrix, distCoeffs=self.distor)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # remove the points which are out of the image
        H = img.shape[0]
        W = img.shape[1]
        proj_img = np.floor(proj_img)
        point_in_image_mask = (proj_img[:, 0] >= 0) & (proj_img[:, 0] < W) & \
                              (proj_img[:, 1] >= 0) & (proj_img[:, 1] < H)

        proj_img = proj_img[point_in_image_mask]
        pointcloud = pointcloud[point_in_image_mask]

        # 强度值至少为1，最大为150
        # index the color of the points
        intensity = (img[np.int_(proj_img[:, 1]), np.int_(proj_img[:, 0])] / 255 * 150).astype(np.uint8)
        intensity[intensity == 0] = 1
        pointcloud = np.column_stack((pointcloud, intensity))

        # visualize
        if debug:
            visualization.o3d_viewer_from_pointcloud(pointcloud, is_normalized=False)

        return pointcloud


if __name__ == '__main__':
    cfg = "sensor.yaml"
    experiment = IntensityRestoration(calib_file=cfg)

    data_dir = Path("/home/helios/Github/workspace/ACSC/data/传感器类型实验/avia")
    pointcloud_img_file_pair = load_pointcloud_img_file_pair(data_dir)
    for idx, pointcloud_file, img_file in tqdm(pointcloud_img_file_pair):
        pointcloud = io.load_pointcloud(str(pointcloud_file))
        img = cv2.imread(str(img_file))
        pointcloud_with_intensity = experiment.append_intensity_to_pc(pointcloud, img, debug=False)
        new_pointcloud_file = str(pointcloud_file.parent / pointcloud_file.stem) + ".pcd"
        io.save_pointcloud(pointcloud_with_intensity, new_pointcloud_file, fields="xyzi")
