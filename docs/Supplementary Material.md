<style>
.markdown-section table {
    width: 60%;
    margin-left: auto;
    margin-right: auto;
}
.markdown-section video {
    width: 60%;
    margin-left: auto;
    margin-right: auto;
    display: flex;
}</style>

# Supplementary Material

## Dataset and setups

We have conducted evaluations using six datasets to assess the effectiveness of our proposed method. The datasets and corresponding sensor suites are listed as follows:

1) `ACSC`: This dataset consists of 18 pairs of correspondences and is provided by ACSC. As shown in Fig.1, the point clouds are collected using the Livox Horizon LiDAR in an indoor laboratory setting.

![](_assets/dataset%20and%20setups/ACSC.png ':size=850 Fig.1 ACSC dataset')

2) `RCLC`: The RCLC dataset comprises 74 pairs of correspondences obtained from RCLC. As shown in Fig.2, the point clouds are collected using the Livox Mid40 LiDAR in a spacious indoor environment.

![](_assets/dataset%20and%20setups/RCLC.png ':size=850 Fig.2 RCLC dataset')

3) `Horizon-exp1`: This dataset consists of 4 pairs of correspondences. As shown in Fig.3, The point clouds are collected by the Livox Horizon LiDAR in a spacious parking lot.

![](_assets/dataset%20and%20setups/Horizon-exp1.png ':size=850 Fig.3 Horizon-exp1 dataset')

4) `Mid70-exp1`: The Mid70-exp1 dataset includes 25 pairs of correspondences. As shown in Fig.4, The point clouds are collected using the Livox Mid70 LiDAR in a cluttered indoor laboratory.

![](_assets/dataset%20and%20setups/Mid70-exp1.png ':size=850 Fig.4 Mid70-exp1 dataset')

5) `Mid70-exp2`: This dataset contains 24 pairs of correspondences. As shown in Fig.5, The point clouds are collected using the Livox Mid70 LiDAR in another cluttered indoor laboratory.

![](_assets/dataset%20and%20setups/Mid70-exp2.png ':size=850 Fig.5 Mid70-exp2 dataset')

6) `Mid70-sim1`: The Mid70-sim1 dataset comprises 28 pairs of correspondences. As shown in Fig.6, The point clouds are collected using the Livox Mid70 LiDAR in a simulated environment.

![](_assets/dataset%20and%20setups/Mid70-sim1.png ':size=850 Fig.6 Mid70-sim1 dataset')

## Existing methods for target segmentation

1) `CloudCompare` is a software designed for processing 3D point clouds. It offers a manual segmentation tool that can be applied in any environment. The human operation cost us 30 seconds. A video is shown as follows.

[Target segmentation by CloudCompare](_media/CloudCompare.mp4 ':include :type=video controls width=70%')

2) `Passthrough filter` is an algorithm that passes through all data that satisfies the user-given constraints. Then the user can remain the LiDAR points $$\mathcal{P} = \{ p | p = (x, y, z) \cap x \in (x_\mathrm{min}, x_\mathrm{max}) \cap  (y_\mathrm{min}, y_\mathrm{max}) \cap z \in (z_\mathrm{min}, z_\mathrm{max}) \}$$, where $$x_\mathrm{min}, x_\mathrm{max}, y_\mathrm{min}, y_\mathrm{max}, z_\mathrm{min}, z_\mathrm{max}$$ are user given parameters. Though this method is simple and can segment the target in arbitrary environments, it requires lots of human attention on parameter tuning. A video clip extracted from [Here](https://www.bilibili.com/video/BV1g24y1W7Td/?vd_source=b5838975531db9548c452816654546d5) is presented below.

[Target segmentation by Passthrough filter](_media/Passthrough%20Filter.mp4 ':include :type=video controls width=70%')

3) To the best of our knowledge, `ACSC` is the pioneering target-based method specifically tailored for non-repetitive scanning LiDAR based on ILCC. This method incorporates a delicate pipeline for segmentation, which encompasses several key steps. Firstly, a passthrough filter is employed to eliminate superfluous and unused point clouds. Next, a statistical outlier removal technique is applied to mitigate noise interference. Additionally, region-growing segmentation is utilized to identify potential clusters corresponding to the targets. A target similarity measurement is subsequently employed to filter out incorrect clusters. Lastly, a height-histogram-based method is implemented to eliminate the holder, assuming that the bottom of the chessboard is parallel to the ground plane. As presented at [https://github.com/HViktorTsoi/ACSC\#22-preparing-the-calibration-board](https://github.com/HViktorTsoi/ACSC\#22-preparing-the-calibration-board), the environment should be well-prepared and costs some time. The abstracted condition can be listed as follows. The checkerboard should be placed on a thin monopod, or suspended in the air with a thin wire. And during the calibration process, the support should be as stable as possible due to the need for point cloud integration; when placing the checkerboard on the base, the lower edge of the board should be parallel to the ground; There are no supposed to be obstructions within 3m of the radius of the calibration board.
4) `RCLC` represents a series of methods that utilize the planner attribute to segment the chessboard point clouds. Firstly, RCLC uses the passthrough filter to filter the cell, floor or wall point clouds. Secondly, it employs RANSAC-based plane fitting to obtain planar objects and eliminate incorrect segments based on the number of LiDAR points and the angle between the normal vector of the target segment plane and the x-axis of the LiDAR frame. It is worth noting that RCLC relies on a well-prepared environment and requires labor-intensive fine-tuning efforts.

## Experiment results of different methods for target segmentation

![](./_assets/results/failure.png ':size=600 Fig.7 Four types of errors for target segmentation')

In our evaluation of the effectiveness of three target segmentation methods (ACSC, RCLC, and ARTS), we have employed two metrics: the number of True Positives (TP) and the TP ratio. The quantitative results are summarized in Table 1 and the qualitative results are presented in Fig.8, Fig.9 and Fig.11. In this evaluation, we have summarized four types of errors that can occur in target segmentation, namely false positives, over-segmentation, under-segmentation, and missing segmentation as shown in Fig. 7.


<table style="border-collapse: collapse; border: none; border-spacing: 0; ">
	<caption>
		TABLE.I The number of TP for 173 pairs of samples from different datasets
	</caption>
	<tr>
		<td rowspan="2" style="border-top: 1px solid black; border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Method
		</td>
		<td colspan="6" style="border-top: 1px solid black; border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Dataset(173)
		</td>
		<td rowspan="2" style="border-top: 1px solid black; border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Ratio of
			<br>
			 TF
		</td>
	</tr>
	<tr>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			ACSC(18)
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			RCLC(74)
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Mid70-exp1(25)
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Mid70-exp2(24)
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Mid70-sim1(28)
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			Horizon-exp1(4)
		</td>
	</tr>
	<tr>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			ACSC
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			18
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			0
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			1
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			2
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			28
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			3
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			0.300
		</td>
	</tr>
	<tr>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			RCLC
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			0
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			74
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			3
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			6
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			1
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			1
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			0.474
		</td>
	</tr>
	<tr>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>Ours</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>18</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>74</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>25</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>24</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>28</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			<b>4</b>
		</td>
		<td style="border-bottom: 1px solid black; text-align: center; padding-right: 3pt; padding-left: 3pt;">
			1.000
		</td>
	</tr>
</table>


According to the result, there are several conclusions listed below.
Firstly, For ACSC and RCLC both have poor performance in our mixed dataset, except in the dataset collected by them. This is because the default parameters are tailored for the dataset, when applying for a new dataset without fine-tuning the parameter the performance of segmentation will break down.

### RCLC

> [!note]
> The majority of the parameters for target segmentation in RCLC remain at their default values, with only a few parameters being modified, like the size of the chessboard.

As depicted in Fig.2, RCLC exhibits poor segmentation performance. The reasons behind this are explained as follows. RCLC adopts an iterative approach to identify potential plane candidates for segmentation. It considers various attributes, including the angle between the normal vector of the chessboard and the LiDAR's z-axis, as well as the number of point clouds, to select suitable chessboard planes. As a result, it restricts the presence of plane-like objects surrounding the chessboard. Moreover, the density of the point cloud varies depending on the type and position of the LiDAR, which may require fine-tuning of the corresponding parameters to achieve accurate segmentation. Additionally, the target segmentation process can lead to under-segmentation, particularly when it comes to accurately segmenting the holder. Furthermore, RCLC incorporates a Gaussian Mixture Model (GMM)-based intensity clustering method to filter out LiDAR points with low intensity. However, this step heavily relies on the initial values, and if not properly set, it may mistakenly filter out points reflected from the black grid of the chessboard, as shown in Dataset (6).

![](./_assets/results/RCLC.png ':size=900')

### ACSC

> [!note]
> The majority of the parameters for target segmentation in ACSC remain at their default values, with only a few parameters being modified, like the size of the chessboard.

![](./_assets/results/ACSC.png ':size=900 Fig.9 Performance of target segmentation of ACSC')

As depicted in Fig.9, ACSC has demonstrated excellent performance in Dataset (1) due to the parameters that were specifically fine-tuned for this dataset.

In Dataset (2). Note that we additionally introduce a passthrough filter module to keep the LiDAR points with coordinate $$z < z_{max}$$, where $$z_{max}=2$$ or the segmentation will break down, specifically, false positives. However, the segmentation performance is still not good enough, the holder has not been segmented out perfectly.

In Dataset (3) and (4), ACSC has the worst performance, including under-segmentation, over-segmentation and false positives. There are three reasons. Firstly, it uses the region growing methods to achieve the chessboard candidate, whose parameters are sensitive to the point cloud density and should be fine-tuned for different datasets. Secondly, it uses a target similarity measurement to filter out incorrect clusters, whose performance depend on a fix parameter about intensity threshold, which also should be fine-tuned. Thirdly, ACSC assume the lower edge of the board should be parallel to the ground, or the holder removal module will break down, resulting in over-segmentation as shown in Fig. 7.

In Dataset (5), ACSC assumes the chessboard with no extra borders around the chessboard (a chessboard with extra border is shown in Fig.10) and have failed to provide the white board removal module, resulting in what we consider as wrong segmentation, specifically, under-segmentation.

![](_assets/dataset%20and%20setups/border.png ':size=300 Fig.10 The chessboard used in RCLC dataset')

Note that ACSC assumes the presence of an upholder to hold the chessboard and present an upholder removal module to remove the holder point clouds. In cases where there is no upholder, the segmentation process may break down. Therefore, in practice, we have excluded the use of this module. Consequently, the segmentation performance has worked well in Dataset (6).

### ARTS (Ours)

As depicted in Figure 11, our proposed algorithm, ARTS, demonstrates excellent performance across all datasets. This result highlights the robustness and consistency of our approach.

![](./_assets/results/Ours.png ':size=900 Fig.11 Performance of target segmentation of ARTS')

## Reference

[1] J. Cui, J. Niu, Z. Ouyang, Y. He, and D. Liu, “Acsc: Automatic calibration for non-repetitive scanning solid-state lidar and camera systems,” arXiv preprint arXiv:2011.08516, 2020. \
[2]  Z. Lai, Y. Wang, S. Guo, X. Meng, J. Li, W. Li, and S. Han, “Laser reflectance feature assisted accurate extrinsic calibration for non-repetitive scanning LiDAR and camera systems,” Opt. Express, vol. 30, no. 10, p. 16242, May 2022.