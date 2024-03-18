from robotmodels.utils.robotmodel import RobotModel

robot_model = RobotModel(robot_name = 'dingo_kinova')
print(f"Absolute urdffile: {robot_model.get_urdf_path()}")
print(f"Absolute xmlfile: {robot_model.get_xml_path()}")
yourdf_model = robot_model.yourdf_model()
home_cfg = robot_model.home_cfg()
yourdf_model.update_cfg(configuration=home_cfg)
yourdf_model.show()

