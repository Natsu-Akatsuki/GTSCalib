# :sparkles: Introduction

<div align="center">
<h4>:rocket: Laziness can lead to productivity</h4>
</div>

This is an extrinsic calibration tool for non-repetitive scanning LiDAR and camera based on a chessboard. The main contribution is to improve the robustness and accuracy of **target segmentation**, and reduce the manual intervention in target-based methods, such as parameter tuning and manual selection of calibration object position.

![](_assets/fig0.png ':size=200 Fig.1 Extrinsic parameters for LiDAR and camera')

This work is an extension of [ACSC](https://github.com/HViktorTsoi/ACSC) and [ILCC](https://github.com/mfxox/ILCC), mainly contributing to the target segmentation module and proposing additional ideas on the intensity threshold segmentation (of Feature extraction module as shown in Fig.2) and PnP algorithm (of Optimization module as shown in Fig.2)

![](_assets/fig1.png ':size=800 Fig.2 A general pipeline for target-based calibration')

## Novel features

- **User-friendly**: No need to adjust parameters, such as dynamically adjusting the passthrough filter to roughly select the position of the calibration board; the calibration board can be handheld or placed on a stool; the bottom edge of the calibration board is not required to be parallel to the ground; and when certain conditions are met, it can be directly placed on the ground.
- **High precision**: Five appropriate positions can obtain high-precision and consistent extrinsic parameters.
- **More general API**: The `Open3D` library is used for point cloud processing, which is easier to install compared to the `python-pcl` library used in `ACSC`.
- **More user-friendly interaction interface**: The `PyQt` and `Rviz` are combined for interaction, providing a more user-friendly data acquisition interface.

## Qusetions and answers

For better demonstrate the benefits our calibration tools, we describe the benefits and significance of this work in a Q&A way.

<details>
    <summary>:question: <b>Question 1：</b>
        Why use target-based segmentation instead of target-less methods?
    </summary>

The target-based methods is more accurate than target-less methods.

</details>

<details>
    <summary>:question: <b>Question 2：</b>
        Why not use the infrastructure-based methods?
    </summary>

The infrastructure-based methods is more accurate than target-based methods, but it requires the installation of additional infrastructure, which is not convenient for field use.

</details>

<details>
    <summary>:question: <b>Question 3：</b>
        How to use CloudCompare to segment the target? (video) (it spends about 30 seconds per frame)
    </summary>

[Target segmentation by CloudCompare](_media/CloudCompare.mp4 ':include :type=video controls width=100%')

</details>

