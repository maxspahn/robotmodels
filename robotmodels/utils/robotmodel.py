from typing import Optional
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
    T = ca.SX.zeros(4, 4)
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

    def fk_casadi(self, q: ca.SX) -> ca.SX:
        """
        Compute the forward kinematics of the robot.
        
        Parameters:
            q (ca.SX): The joint configuration.
        
        Returns:
            ca.SX: The homogeneous transformation matrix of the end-effector.
        """
        T = ca.SX.eye(4)
        for i, joint in enumerate(self._joints):
            T = ca.mtimes(T, joint.transformation_matrix(q[i]))
        return T

    def parse_xml(self):
        self._joints = []
        self._bodies = []
        self.tree = ET.parse(self.get_xml_path())
        root = self.tree.getroot()
        self.root = root.find('.//worldbody')
        self.parse_geom()
        self.parse_joints()



    def parse_geom(self):
        geoms = self.root.findall('.//geom')
        geom_data = {}
        for i, geom in enumerate(geoms):
            if not 'type' in geom.attrib:
                continue
            if 'name' in geom.attrib:
                geom_id = geom.attrib['name']
            else:
                geom_id = 'geom_' + str(i)
            geom_type = geom.attrib['type']
            self._bodies.append(Body(geom_id, geom_type))

    def parse_joints(self):
        joints = self.root.findall('.//joint')
        joint_data = {}
        for i, joint in enumerate(joints):
            if not 'type' in joint.attrib:
                joint_type = 'hinge'
            else:
                joint_type = joint.attrib['type']

            if not 'name' in joint.attrib:
                joint_id = 'joint_' + str(i)
            else:
                joint_id = joint.attrib['name']
            if not 'axis' in joint.attrib:
                axis = [0, 0, 1]
            else:
                axis = [float(x) for x in joint.attrib['axis'].split()]
            print(joint_type)
            print(joint_id)
            self._joints.append(Joint(joint_id, axis, joint_type_map_xml[joint_type]))
        breakpoint()



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
