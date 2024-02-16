from typing import Optional
import os
import shutil
import importlib.resources
import numpy as np
import yourdfpy
from yourdfpy.urdf import URDF
import robotmodels


class RobotModel():
    """General class to handle loading models."""
    _robot_name: str
    _model_name: Optional[str]

    def __init__(self, robot_name: str, model_name: Optional[str]= None):
        """
        Parameters:
            robot_name (str): The name of the robot.
            model_name (str): The mode of the robot.
        """
        self._robot_name = robot_name
        self._model_name = model_name

    def copy_model(self, path: str) -> None:
        """Copies the entire robot model to the current working directory."""
        model_path = os.path.join(robotmodels.__path__[0], self._robot_name)
        shutil.copytree(model_path, path)

    @property
    def package_name(self) -> str:
        return f"robotmodels.{self._robot_name}"

    @property
    def urdf_package_name(self) -> str:
        return f"robotmodels.{self._robot_name}.urdf"

    @property
    def xml_package_name(self) -> str:
        return f"robotmodels.{self._robot_name}.xml"

    def get_urdf_path(self) -> str:
        """
        Get the path to the URDF file for the specified robot.
        
        Returns:
            str: The absolute path to the URDF file.
        """
        if self._model_name:
            resource_name = f"{self._model_name}.urdf"
        else:
            resource_name = f"{self._robot_name}.urdf"
        with importlib.resources.path(self.urdf_package_name, resource_name) as urdf_path:
            return str(urdf_path)

    def get_xml_path(self) -> str:
        """
        Get the path to the URDF file for the specified robot.
        
        Returns:
            str: The absolute path to the URDF file.
        """
        if self._model_name:
            resource_name = f"{self._model_name}.xml"
        else:
            resource_name = f"{self._robot_name}.xml"
        with importlib.resources.path(self.xml_package_name, resource_name) as urdf_path:
            return str(urdf_path)

    def yourdf_model(self) -> URDF:
        """
        Get the yourdfy model.

        This can be used to get all sorts of properties, such as forward
        kinematics, degrees of freedom.
        """
        urdf_file = self.get_urdf_path()
        return yourdfpy.URDF.load(urdf_file)


    def center_cfg(self) -> np.ndarray:
        """
        Returns the center configuration.
        """
        urdf_file = self.get_urdf_path()
        urdf_model = yourdfpy.URDF.load(urdf_file)
        return urdf_model.center_cfg

    def home_cfg(self) -> np.ndarray:
        """
        Returns the home configuration defined in robotmodels.
        """
        if self._model_name:
            resource_name = f"{self._model_name}_home.txt"
        else:
            resource_name = f"{self._robot_name}_home.txt"
        try:
            with importlib.resources.path(self.package_name, resource_name) as home_file:
                home_configuration_file = str(home_file)
            return np.loadtxt(home_configuration_file)
        except FileNotFoundError:
            print("No home configuration specified in robotmodels.")
            print("Using center configuration instead.")
            return self.center_cfg()


class LocalRobotModel(RobotModel):
    """LocalRobotModel uses the current working directory to search
    for the robotmodel folder."""
    @property
    def package_name(self) -> str:
        return f"{self._robot_name}"

    @property
    def urdf_package_name(self) -> str:
        return f"{self._robot_name}.urdf"

    @property
    def xml_package_name(self) -> str:
        return f"{self._robot_name}.xml"
