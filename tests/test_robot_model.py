import os

import numpy as np

from robotmodels.utils.robotmodel import RobotModel

ROBOT_NAMES_URDF = [
    'panda',
    'ur5',
    'albert',
    'iris',
    'prius',
    'tiago',
    'iiwa7',
    ('kinova','gen3lite'),
    ('kinova','gen3_6dof'),
    ('kinova','gen3_7dof'),
]
ROBOT_NAMES_XML = [
    ('panda', 'panda'),
    ('kinova', 'gen3lite'),
]

def test_urdf_files():
    for robot_name in ROBOT_NAMES_URDF:
        if isinstance(robot_name, tuple):
            robot_model = RobotModel(robot_name = robot_name[0], model_name = robot_name[1])
        else:
            robot_model = RobotModel(robot_name = robot_name)
        urdf_file = robot_model.get_urdf_path()
        assert isinstance(urdf_file, str)
        assert os.path.exists(urdf_file)

def test_xml_files():
    for robot_name in ROBOT_NAMES_XML:
        if isinstance(robot_name, tuple):
            robot_model = RobotModel(robot_name = robot_name[0], model_name = robot_name[1])
        else:
            robot_model = RobotModel(robot_name = robot_name)
        xml_file = robot_model.get_xml_path()
        assert isinstance(xml_file, str)
        assert os.path.exists(xml_file)

def test_default_configurations():
    for robot_name in ROBOT_NAMES_URDF:
        if isinstance(robot_name, tuple):
            robot_model = RobotModel(robot_name = robot_name[0], model_name = robot_name[1])
        else:
            robot_model = RobotModel(robot_name = robot_name)
        center_configuration = robot_model.center_cfg()
        home_configuration = robot_model.home_cfg()
        assert isinstance(center_configuration, np.ndarray)
        assert isinstance(home_configuration, np.ndarray)

