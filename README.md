# RobotModels

RobotModels is a simple package providing robot description as urdf or xml for
mujoco. It also provides a visualization using yourdfpy.

## Installation

```bash
pip install robotmodels
```

## Usage

You can create a robotmodel based on the name. 
Then, you can visualize it and get access to the urdf and the xml.

```python3
from robotmodels.utils.robotmodel import RobotModel

robot_model = RobotModel(robot_name = 'panda')
print(f"Absolute urdffile: {robot_model.get_urdf_path()}")
print(f"Absolute xmlfile: {robot_model.get_xml_path()}")
yourdf_model = robot_model.yourdf_model()
home_cfg = robot_model.home_cfg()
yourdf_model.update_cfg(configuration=home_cfg)
yourdf_model.show()
```
