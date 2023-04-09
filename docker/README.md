# ACSC

```bash
$ docker pull osrf/ros:melodic-desktop-full
$ docker run -it --privileged --name=acsc melodic-desktop-full

# 安装pybind封装库
(docker-root)$ git clone --recurse-submodules https://github.com/HViktorTsoi/ACSC
(docker:root)$ cd ACSC/segmentation/
(docker:root)$ python setup.py install

# 安装依赖
(docker:root)$ apt install python-pip unzip vim python-tk
(docker:root)$ pip install --upgrade pip
(docker:root)$ pip install gdown opencv-python==4.0.0.21 scipy scikit-learn transforms3d pyyaml==5.4.1

# 安装python-pcl依赖
(docker:root)$ cd /
(docker:root)$ git clone https://github.com/strawlab/python-pcl.git
# 使用https://github.com/strawlab/python-pcl/issues/307来修订setup.py
(docker:root)$ ...
(docker:root)$ python setup.py install

# 导入数据集
(docker:root)$ cd ACSC
(docker:root)$ mkdir data && cd data
(docker:root)$ gdown 1hgV-aH_RSsL1geHDSIlwwvqrMy6ED56K
(docker:root)$ unzip sample.zip
(docker:root)$ rm sample.zip 

# 执行标定
(docker:root)$ python calibration.py --config ./configs/sample.yaml
```

