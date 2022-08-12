# Version of the library that will be used to upload to pypi
__version__ = "1.0.1"


# For videos only

class Env:
    def __init__(self):
        pass

    def set_camera_pose(self, pose):
        pass

    def load_robot(self, robot_name, pose_sampler='default', pos=None, orn=None):
        pass

    def robots_random_move(self,):
        pass

    def camera_synthesis(self, return_annos):
        pass

    def domain_randomization(self,):
        pass


def connect(ip, port):
    return Env()


def sample_camera_pose(env):
    return 1


def set_camera_pose():
    pass


def load_flexiv():
    pass


def load_kuka_iiwa():
    pass


def load_franka():
    pass


def load_ur5():
    pass


def load_tobor():
    pass


def robots_random_move():
    pass


def camera_synthesis():
    pass


def domain_randomization():
    pass
