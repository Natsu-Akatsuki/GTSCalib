# :sparkles: Introduction

<div align="center">
<h4>:rocket: Laziness can lead to productivity</h4>
</div>

This is an extrinsic calibration tool for non-repetitive scanning LiDAR and camera based on a chessboard. The main contribution is to improve the robustness and accuracy of **target segmentation**, and reduce the manual intervention in target-based methods, such as parameter tuning and manual selection of calibration object position.

![](_assets/fig0.png ':size=200 Fig.1 Extrinsic parameters for LiDAR and camera')

This work is an extension of [ACSC](https://github.com/HViktorTsoi/ACSC) and [ILCC](https://github.com/mfxox/ILCC), mainly contributing to the target segmentation module and proposing additional ideas on the intensity threshold segmentation (of Feature extraction module as shown in Fig.2) and PnP algorithm (of Optimization module as shown in Fig.2)

![](_assets/fig1.png ':size=800 Fig.2 A general pipeline for target-based calibration')

## Novel features

- **User-friendly**:
  - Free of parameter tuning: Our proposed method eliminates the need for adjusting parameters, such as dynamically fine-tuning the passthrough filter to approximate the position of the calibration board. It operates effectively without the need for such manual adjustments, making the calibration process more convenient and user-friendly.
  - Environment agnostic: Our proposed method does not require a well-prepared environment. It can effectively function in spaces with clustered backgrounds. The calibration board can be handheld or placed on a stool, and there is no strict requirement for the bottom edge of the calibration board to be parallel to the ground. Under certain conditions, the calibration board can even be directly placed on the ground for calibration.

![img](_assets/segmentation.png ':size=800 Fig.3 Performance of the target segmentation. Scene1, Scene 3 and Scene 4: the chessboard is held by a holder; Scene2: the chessboard is handheld; Scene5: the chessboard is placed on a chair; Scene6: the chessboard is placed on the ground')

- **High precision**: Five appropriate positions can obtain high-precision extrinsic parameters.
- **More general API**: The `Open3D` library is used for point cloud processing, which is easier to install compared to the `python-pcl` library used in `ACSC`.
- **More user-friendly interaction interface**: The `PyQt` and `Rviz` are combined for interaction, providing a more user-friendly data acquisition interface.

## Qusetions and answers

For better demonstrate the benefits our calibration tools, we describe the benefits and significance of this work in a Q&A way.

<details>
    <summary>:question: <b>Question 1：</b>
        Why use target-based segmentation instead of target-less methods?
    </summary>

The target-based methods is more accurate than targetless methods.

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

