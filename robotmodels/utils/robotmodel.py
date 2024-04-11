from typing import Optional, Union
import os
import shutil
import sys
from typing import List
import xml.etree.ElementTree as ET
import casadi as ca
import importlib.resources
import numpy as np
import yourdfpy
from yourdfpy.urdf import URDF
import robotmodels

joint_type_map_xml = {
    'hinge': 'revolute',
    'slide': 'prismatic',
}


def rotation_matrix(axis: List[int], theta: ca.SX):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = ca.SX(axis)
    axis = axis / ca.sqrt(ca.dot(axis, axis))
    a = ca.cos(theta / 2.0)
    b, c, d = ca.vertsplit(-axis * ca.sin(theta / 2.0))
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    R = ca.vertcat(ca.horzcat(aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)),
                   ca.horzcat(2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)),
                   ca.horzcat(2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc))
    return R

def homogeneous_transformation(rotation_matrix: ca.SX, translation: ca.SX) -> ca.SX:
    """
    Construct a homogeneous transformation matrix from rotation matrix and translation vector.
    """
    T = ca.SX.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = translation
    return T

class Joint():
    def __init__(self, name: str, axis: List[int], type: str):
        self.name = name
        self.axis = axis
        self.type = type

    def transformation_matrix(self, joint_value: ca.SX):
        if self.type == 'revolute':
            return homogeneous_transformation(rotation_matrix(self.axis, joint_value), ca.SX.zeros(3))
        if self.type == 'prismatic':
            return homogeneous_transformation(ca.SX.eye(3), self.axis * joint_value)
        else:
            return ca.SX.eye(4)

class Body():
    def __init__(self, name: str, body_type: str):
        self.name = name
        self.body_type = body_type

class RobotModel():
    """General class to handle loading models."""
    _robot_name: str
    _model_name: Optional[str]
    _joints: List[Joint]
    _bodies: List[Body]

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

    def fk_casadi(self, q: ca.SX, child_link: str, parent_link: Union[str, None] = None, link_transformation=np.eye(4), position_only=False) -> ca.SX:
        """
        Compute the forward kinematics of the robot.
        
        Parameters:
            q (ca.SX): The joint configuration.
        
        Returns:
            ca.SX: The homogeneous transformation matrix of the end-effector.
        """
        T = ca.SX.eye(4)
        joint_counter = 0
        child_link_found = False
        for element in self.root.iter():
            if child_link_found:
                return T
            T_element = ca.SX.eye(4)
            if element.tag == 'body':
                pos = element.get('pos') if 'pos' in element.attrib else '0 0 0'
                offset = ca.SX([float(i) for i in pos.split()])
                T_element = homogeneous_transformation(ca.SX.eye(3), offset)
                if element.get('name') == child_link:
                    child_link_found = True
            if element.tag == 'joint':
                joint_type = 'hinge' if not 'type' in element.attrib else element.attrib['type']
                joint_axis = [0, 0, 1] if not 'axis' in element.attrib else [int(x) for x in element.attrib['axis'].split()]
                if joint_type == 'hinge':
                    T_element = homogeneous_transformation(
                        rotation_matrix(joint_axis, q[joint_counter]),
                        ca.SX.zeros(3)
                    )
                elif joint_type == 'slide':
                    T_element = homogeneous_transformation(
                        ca.SX.eye(3),
                        joint_axis * q[joint_counter]
                    )
                joint_counter += 1
            if element.tag == 'geom':
                if element.get('name') == child_link:
                    pos = element.get('pos') if 'pos' in element.attrib else '0 0 0'
                    offset = ca.SX([float(i) for i in pos.split()])
                    T_element = homogeneous_transformation(ca.SX.eye(3), offset)
                    child_link_found = True
            T = ca.mtimes(T, T_element)
        if not child_link_found:
            raise ValueError(f"Child link {child_link} not found in XML.")
        else:
            return T


    def parse_xml(self):
        self._joints = []
        self._bodies = []
        self.tree = ET.parse(self.get_xml_path())
        root = self.tree.getroot()
        self.root = root.find('.//worldbody')

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
