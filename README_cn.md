# ARTSCalib

<div align="center">

[English](README.md) | 简体中文
</div>

## 1. 简介

适用于基于**棋盘格标定板**的**非重复扫描激光雷达和相机**的**外参标定**工具。主要贡献为基于image-based
representation提高了分割的鲁棒性和准确性，减少target-based方法中人为的干预，例如调参和手动框选标定物位置。该工作为[ACSC](https://github.com/HViktorTsoi/ACSC)
和[ILCC](https://github.com/mfxox/ILCC)的拓展，主要对棋盘格标定分割作为一些贡献和对其中强度阈值分割和PnP算法提出了一些思考。具有如下特性：

- **用户友好**：基本不用调参，比如不用动态调整直通滤波来框选出标定板的大致位置；标定板可以不用三脚架支撑，想手持就手持，想放凳子上就放凳子上；也不要求标定板底边平行于地面；当满足某些条件时，还能直接放地面。
- **高精度**：五个适当的位置，即能得到较高精度的外参；
- **更通用的API**：代码实现上，使用`Open3D`库进行点云处理，相比于`ACSC`移除不再维护和比较难安装的`python-pcl`库；
- **更友好的交互界面**：结合`PyQt`和`Rviz`进行交互，提供更友好的数据采集界面。

## 2. 依赖

- ROS

|           —            |       数据采集程序       |       外参标定程序       |         仿真         |
|:----------------------:|:------------------:|:------------------:|:------------------:|
| Ubuntu20.04 ROS Noetic | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| Ubuntu22.04 ROS Humble |        :x:         | :heavy_check_mark: |        :x:         |

> **Note**
>
> （1）UI程序基于ROS RViz的Python拓展库开发，则**当前调用环境的Python版本需与封装时的Python版本一致**
> 。如Melodic对应的是2.7版本的Python。Noetic对应的是3.8的Python。若不一致则需通过源码编译Python拓展库并导入相关环境变量，不建议不熟练该环境配置的人使用。
>
> （2）若使用conda环境，则使用的Python解释器也应该有所对应。否则同（1）

- Python

```bash
$ git clone https://github.com/Natsu-Akatsuki/ARTSCalib
$ pip3 install -r requirements.txt

# 使用采集数据的UI时需要使用额外安装：
$ pip install PyQt5 rospkg rospy
```

- [PointCloud-PyUsage](https://github.com/Natsu-Akatsuki/PointCloud-PyUsage)（为自定义的点云处理库）

```bash
$ pip3 install -U --user build pip setuptools wheel
$ sudo apt install pybind11-dev
$ git clone https://github.com/Natsu-Akatsuki/PointCloud-PyUsage --depth=1
$ cd PointCloud-PyUsage
$ bash install.sh
```

## 3. 棋盘格标定板

之前应该是用泡沫板的棋盘格，然后黑白格子的强度区分度不大。后面是专门定制了，标定板材料为雪弗板，`5cm`厚，黑色UV，8×5，0.08m。

<p align="center">
<img src="docs/target.png" alt="img" width=67% style="zoom:67%"/>
</p>

## 4. 步骤

步骤一：获取相机内参和畸变系数（如使用Matlab标定工具）

步骤二：采集数据

（1）若基于UI：

- 启动激光雷达和相机驱动的相关节点

```bash
# 例如：
$ roslaunch livox_ros_driver livox_lidar.launch
```

- 启动采集界面（暂无通过18.04下的测试），支持采集`xyzi`格式的点云数据

```bash
$ cd ui
# 步骤一：修改ui/data_collection.yaml中的参数（e.g. ROS的主题，棋盘格的pattern，是否对图片进行水平翻转等）
# 步骤二：启动程序
$ python ui/ui.py
```

<img src="docs/ui.png" alt="image-20230225151555953"  />

> **Note**
>
>Record Image：纯粹采集图片
> Record Pointcloud & Image：采集积分后的点云和采集时的第一帧图片
> isImageReady：判断能否从图片上检测出2D角点

<img src="docs/calibration_tools.gif" alt="image-20220227223539620" style="zoom:80%;" />

> **Note**
>
> 导出的图片或反色，调用Python OpenCV的API，rgb2bgr即可

```bash
$ python3 ui.py
```

导出的数据文件夹为如下，同时在该目录参考`sensor-template.yaml`创建sensor.yaml文件并填写相机的内参和畸变系数（外参部分用于占位，可忽视）

```bash
.data
├── 04-09-12-19-29
│   ├── img
│   ├── img_for_intrinsic
│   ├── pointcloud
│   └── sensor.yaml <-需参考sensor-template.yaml自行添加
└── sensor-config.yaml
```

（2）若其他方法得到的数据，则需按如下方式构建目录树

```bash
├── data
│   ├── 目录名
│   │   ├── img
│   │   │   ├── 000000.png
│   │   │   ├── 000001.png
│   │   │   ├── 000002.png
│   │   │   ├── 000003.png
│   │   ├── pointcloud
│   │   │   ├── 000000.pcd
│   │   │   ├── 000001.pcd
│   │   │   ├── 000002.pcd
│   │   │   ├── 000003.pcd
│   │   └── sensor.yaml
```

步骤三：根据数据集的路径，传感器的类型，棋盘格的类型，调整配置文档`config/config.yaml`的参数

<img src="docs/cfg.png" alt="image-20230409211529712" style="zoom: 80%;" />

```bash
$ python3 calibration_node.py -cfg config/config.yaml
# 生成的外参位于sensor.yaml
```

步骤四：执行定性分析和定量分析

```bash
$ python3 experiment.py -cfg config/config.yaml
```

<img src="docs/color_pc.png" alt="image-20230409213203028" style="zoom:67%;" />

## 5. 其他用例（To be continued）

### 5.1 仿真（待补充代码）

```bash
$ cd simulation/livox_simulation/
$ catkin build
$ source devel/setup.bash

# For avia
$ roslaunch livox_laser_simulation avia_camera_calibration.launch
# For horizon
$ roslaunch livox_laser_simulation horizon_camera_calibration.launch
# For mid40
$ roslaunch livox_laser_simulation mid40_camera_calibration.launch
# For mid70
$ roslaunch livox_laser_simulation mid70_camera_calibration.launch

# For developer (docker user)
(docker) $ export DISPLAY=:0
(docker) $ __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia roslaunch livox_laser_simulation mid70_camera_calibration.launch
```

![image-20230225145116576](docs/simulation.png)

### 5.3 标定板分割

```bash
# 对特定的点云文件进行分割，可通过打断点看效果
# 对第0和1帧点云进行分割
$ python3 target_segmentation.py --cfg config/horizon.yaml --idx 0 1
```



