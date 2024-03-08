import sys
from robotmodels.utils.robotmodel import RobotModel

def main():
    robot_name = 'kinova'
    model_name = 'gen3lite'
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    if len(sys.argv) > 2:
        model_name = sys.argv[2]
    robot_model = RobotModel(robot_name = robot_name, model_name = model_name)
    print(f"Absolute urdffile: {robot_model.get_urdf_path()}")
    yourdf_model = robot_model.yourdf_model()
    home_cfg = robot_model.home_cfg()
    yourdf_model.update_cfg(configuration=home_cfg)
    yourdf_model.show()

if __name__ == "__main__":
    main()
