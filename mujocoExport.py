# FUSION 360 SCRIPT
# DESCRIPTION: Exports a Fusion 360 design to a MuJoCo-compatible XML file.
# This script uses an object-oriented approach to export each component as an
# STL mesh and create an XML file with joints positioned according to the
# Fusion 360 model.

import adsk.core, adsk.fusion, adsk.cam, traceback
import os
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

class MujocoExporter:
    """
    A class to handle the logic of exporting a Fusion 360 design
    to a MuJoCo XML file.
    """
    def __init__(self, root_comp_name, export_folder):
        # --- Initialize Fusion 360 Application and UI ---
        self.app = adsk.core.Application.get()
        self.ui = self.app.userInterface
        self.design = self.app.activeProduct
        if not self.design:
            raise RuntimeError("No active design found.")
        
        self.root_comp = self.design.rootComponent
        
        # --- Find the specified root component occurrence (the base link) ---
        self.base_link_occurrence = self._find_base_link(root_comp_name)
        if not self.base_link_occurrence:
            raise RuntimeError(f"Component '{root_comp_name}' not found as a direct occurrence in the root.")
            
        # --- Setup paths and folders ---
        self.export_folder = export_folder
        self.mesh_folder = os.path.join(self.export_folder, 'meshes')
        if not os.path.exists(self.mesh_folder):
            os.makedirs(self.mesh_folder)
            
        # --- Initialize state variables ---
        self.seen_links = set() # To avoid processing the same component multiple times
        
        # --- Initialize XML structure ---
        self.mujoco_root = Element('mujoco', model=self.root_comp.name.replace(" ", "_"))
        self.asset_element = None
        self.worldbody_element = None
        self.actuator_element = None

    def _find_base_link(self, root_comp_name):
        """Finds the first occurrence of a component with the given name."""
        for occ in self.root_comp.occurrences:
            if occ.component.name == root_comp_name:
                return occ
        return None

    def _initialize_xml(self):
        """Sets up the basic structure of the MuJoCo XML file."""
        default_element = SubElement(self.mujoco_root, 'default')
        SubElement(default_element, 'mesh', scale="0.001 0.001 0.001") # Convert from Fusion's internal cm to m
        self.asset_element = SubElement(self.mujoco_root, 'asset')
        self.actuator_element = SubElement(self.mujoco_root, 'actuator')
        self.worldbody_element = SubElement(self.mujoco_root, 'worldbody')
        
        # Add a world plane
        SubElement(self.worldbody_element, 'geom', type='plane', size='1 1 0.1')

    def export(self):
        """Main export function."""
        self._initialize_xml()
        
        # --- Create the base link body ---
        base_link_name = self.base_link_occurrence.component.name.replace(':', '_').replace(' ', '_')
        base_link_body_element = SubElement(self.worldbody_element, 'body', name=base_link_name, pos='0 0 .139')
        SubElement(base_link_body_element, 'freejoint', name="root")
        
        # Export the mesh for the base link and add it to the set of seen links
        self._export_mesh(self.base_link_occurrence, base_link_body_element)
        self.seen_links.add(self.base_link_occurrence.fullPathName)

        # --- Recursively build the robot tree starting from the base link ---
        self._build_robot_tree(self.base_link_occurrence, base_link_body_element)
        
        # --- Write the final XML file ---
        self._write_xml_file()
        
        self.ui.messageBox(f'Successfully exported to {self.export_folder}')

    def _get_component_links(self, parent_occurrence): 
        all_joints = []
        parent = parent_occurrence.assemblyContext
        while parent.assemblyContext and len(parent.joints) == 0:
            parent = parent.assemblyContext
        parent_comp = parent.component
        
        for joint in parent_comp.joints:
            all_joints.append(joint)
        for joint in parent_comp.asBuiltJoints:
            all_joints.append(joint)

        found_joints = []
        for joint in all_joints:
            if not joint.occurrenceTwo:
                continue
            print(joint.occurrenceOne.fullPathName)
            if joint.occurrenceOne == parent_occurrence or joint.occurrenceTwo == parent_occurrence:
                found_joints.append(joint)
        return found_joints


    def _build_robot_tree(self, parent_occurrence, parent_xml_element):
        """
        Recursively finds child components connected by joints and builds the XML structure.
        This version iterates through joints attached to the parent occurrence.
        """

        print(f"get joints {parent_occurrence.fullPathName}")
        # Combine the joints and as-built joints for the current occurrence's component
        all_connected_joints = []
        try:
            for joint in parent_occurrence.joints:
                all_connected_joints.append(joint)
            for joint in parent_occurrence.asBuiltJoints:
                all_connected_joints.append(joint)
        except RuntimeError as e:
            print(e)
            print(f"error getting {parent_occurrence.fullPathName} joints, trying fallback")
            all_connected_joints = self._get_component_links(parent_occurrence)

        if len(all_connected_joints) == 0:
            all_connected_joints = self._get_component_links(parent_occurrence)


        print(f"len joints {len(all_connected_joints)}")
        for joint in all_connected_joints:
            if not joint.occurrenceTwo:
                continue

            # Determine which occurrence is the parent and which is the child
            if joint.occurrenceOne == parent_occurrence:
                child_occurrence = joint.occurrenceTwo
            elif joint.occurrenceTwo == parent_occurrence:
                child_occurrence = joint.occurrenceOne
            else:
                # This joint does not connect to the current parent component, so skip it.
                continue

            if child_occurrence.entityToken in self.seen_links:
                continue # Already processed this link

            self.seen_links.add(child_occurrence.entityToken)
            
            # --- Create Child Body in XML ---
            child_body_name = child_occurrence.fullPathName.replace(':', '_').replace(' ', '_')
            child_body_element = SubElement(parent_xml_element, 'body', name=child_body_name)

            # --- Get Joint Information ---
            if isinstance(joint, adsk.fusion.Joint):
                # Get the joint's origin relative to the child component
                joint_origin = joint.geometryOrOriginTwo.origin if joint.occurrenceTwo == child_occurrence else joint.geometryOrOriginOne.origin
                
                # Transform the joint origin from the component's space to the parent's space
                transform = parent_occurrence.transform
                transform.invert()
                joint_origin.transformBy(transform)
                joint_origin.transformBy(child_occurrence.transform)

                # Convert from Fusion's cm to MuJoCo's meters
                pos_str = f'{joint_origin.x / 100.0:.4f} {joint_origin.y / 100.0:.4f} {joint_origin.z / 100.0:.4f}'
                child_body_element.set('pos', pos_str)
                
                # --- Create Joint inside Child Body ---
                if joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType:
                    joint_name = joint.name.replace(' ', '_')
                    SubElement(child_body_element, 'joint', 
                             name=joint_name, 
                             type=self._get_mujoco_joint_type(joint.jointMotion.jointType),
                             axis='0 0 1') # You may need to derive the correct axis
                    
                    # Add a corresponding actuator for the joint
                    SubElement(self.actuator_element, "position", name=joint_name, joint=joint_name)

            # --- Export Mesh for the Child and recurse ---
            self._export_mesh(child_occurrence, child_body_element)
            self._build_robot_tree(child_occurrence, child_body_element)

    def _export_mesh(self, occurrence, body_element):
        """Exports a component's mesh as STL and adds the corresponding XML tags."""
        comp_name = occurrence.fullPathName.replace(':', '_').replace(' ', '_')
        mesh_asset_name = f'{comp_name}_mesh'
        mesh_filename = f"{comp_name}.stl"
        mesh_filepath = os.path.join(self.mesh_folder, mesh_filename)

        export_manager = self.design.exportManager
        
        # Pass the occurrence directly. This exports the component
        # in its correct position within the assembly.
        stl_options = export_manager.createSTLExportOptions(occurrence, mesh_filepath)
        
        stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
        export_manager.execute(stl_options)

        # Add mesh to the asset section
        SubElement(self.asset_element, 'mesh', file=f'meshes/{mesh_filename}', name=mesh_asset_name)

        # Add a geom pointing to the mesh asset
        SubElement(body_element, 'geom', type='mesh', mesh=mesh_asset_name)
    
    @staticmethod
    def _get_mujoco_joint_type(fusion_joint_type):
        """Maps Fusion 360 joint types to MuJoCo joint types."""
        if fusion_joint_type == adsk.fusion.JointTypes.RevoluteJointType:
            return 'hinge'
        elif fusion_joint_type == adsk.fusion.JointTypes.SliderJointType:
            return 'slide'
        elif fusion_joint_type == adsk.fusion.JointTypes.BallJointType:
            return 'ball'
        else:
            return 'hinge' # Default for other types

    def _write_xml_file(self):
        """Formats and writes the final XML file."""
        xml_string = tostring(self.mujoco_root, 'utf-8')
        reparsed = minidom.parseString(xml_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")
        
        xml_file_path = os.path.join(self.export_folder, self.root_comp.name.replace(" ", "_") + '.xml')
        with open(xml_file_path, "w") as f:
            f.write(pretty_xml)


def run(context):
    """The main entry point for the Fusion 360 script."""
    ui = None
    try:
        # --- User Inputs (hardcoded for simplicity) ---
        root_comp_name = 'body' 
        export_folder = '/Users/shpurcell/Documents/code/mujoco_test_export' # CHANGE THIS PATH

        # --- Instantiate the exporter and run the export process ---
        exporter = MujocoExporter(root_comp_name, export_folder)
        exporter.export()

    except Exception as e:
        app = adsk.core.Application.get()
        ui = app.userInterface
        if ui:
            ui.messageBox('Failed:\n{}\n{}'.format(str(e), traceback.format_exc()))