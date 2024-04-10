import casadi as ca
from robotmodels.utils.robotmodel import RobotModel

robot_model = RobotModel(robot_name = 'nlink', model_name='nlink_2')
robot_model.parse_xml()
q_ca = ca.SX.sym('q_ca', 9)
fk = robot_model.fk_casadi(q_ca)
print(fk)

