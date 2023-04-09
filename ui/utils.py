import pickle
import signal
from multiprocessing import Process

import cv2
import numpy as np
import roslaunch
import rospy
import yaml
import rosgraph


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


class RosLaunchProc(Process):
    def __init__(self, args):
        """创建一个进程来启动roslaunch文件"""
        super().__init__()
        self.launch_proc = None
        self.roslaunch_files = args[0]
        self.is_core = args[1]

    def shutdown_handle(self, sig, frame):
        """
        自定义信号回调函数调用shutdown，调用后将disable launch spin，使进程terminate
        """
        self.launch_proc.shutdown()
        rospy.loginfo(f"\033[1;31m成功调用launch.shutdown()\033[0m")

    def run(self):
        signal.signal(signal.SIGUSR1, self.shutdown_handle)
        self.launch_proc = self.roslaunch_api()
        self.launch_proc.start()
        # 阻塞，防止进程stop状态
        try:
            self.launch_proc.spin()
        finally:
            pass

    def roslaunch_api(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
        roslaunch.configure_logging(uuid)
        return roslaunch.parent.ROSLaunchParent(uuid, self.roslaunch_files, self.is_core)


def launch_roscore():
    if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
        pass
    else:
        print("\033[1;32m\n自检(0)：roscore未启动，将启动roscore\n\033[0m")
        roscore = RosLaunchProc(([], False))
        roscore.roslaunch_api().start()
