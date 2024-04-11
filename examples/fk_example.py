import casadi as ca
import numpy as np
from robotmodels.utils.robotmodel import RobotModel

robot_model = RobotModel(robot_name = 'panda', model_name='panda')
robot_model = RobotModel(robot_name = 'nlink', model_name='nlink_2')
robot_model.parse_xml()
q_ca = ca.SX.sym('q_ca', 7)
fk = robot_model.fk_casadi(q_ca, 'panda_link3')
fk_fun = ca.Function('fk', [q_ca], [fk])
q_np = np.zeros(7)
fk_np = fk_fun(q_np)
print(fk_np)

