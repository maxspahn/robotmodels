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

In case, you want to make local changes to the robotmode, you can copy the 
robotmodel to a local path using 

```python3
from robotmodels.utils.robotmodel import RobotModel

robot_model = RobotModel(robot_name = 'panda')
robot_model.copy_model(path = 'path/to/your/local/folder')
```

Now, you can make local changes to the robot.
Then you can load it using:
```python3
from robotmodels.utils.robotmodel import RobotModel, LocalRobotModel
robot_model = LocalRobotModel(robot_name = 'name_folder')
```

Note that `name_folder` must be in the current working directory.

## Adding a new robotmodel

If you want to add new robotmodel, you should copy the template folder to a new
folder with the name of the robotmodel. Then, you can change the urdf, xml and
the meshes.

**Mesh locations in the urdf file must be adapted to the paths**
**where the meshes are stored. Mostly, ros-package syntax is not suppoorted.**

Ideally, you could also add the new class to the unittests in the `tests`
folder.

To verify your new model, install the new robotmodel version (`poetry version
patch` to increase the version number) and run the tests. Alternatively you can
also use the examples to visualize the robot model.


