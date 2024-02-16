import yourdfpy
import os
import sys
import numpy as np
from robotmodels.utils.robotmodel import LocalRobotModel

def main():
    robot_name = 'panda'
    model_name = None
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    if len(sys.argv) > 2:
        model_name = sys.argv[2]
    robot_model = LocalRobotModel(robot_name = robot_name, model_name = model_name)
    urdf_file = robot_model.get_urdf_path()
    yourdf_model = robot_model.yourdf_model()

    center_cf = yourdf_model.center_cfg
    home_cfg = robot_model.home_cfg()

    urdf_model = yourdfpy.URDF.load(urdf_file)
    urdf_model.update_cfg(configuration=home_cfg)
    urdf_model.show()

if __name__ == "__main__":
    main()
