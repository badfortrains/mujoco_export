# FUSION 360 SCRIPT
# DESCRIPTION: Exports a Fusion 360 design to a MuJoCo-compatible XML file.
# This script exports each component as an STL mesh and creates an XML file
# with joints positioned according to the Fusion 360 model.

import adsk.core, adsk.fusion, adsk.cam, traceback
import os
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

_overrides = {
    'robot_v5(Mirror)_1+Motor_mount_v5(Mirror)_1+mount(Mirror)_1Revolute_1': {
        'axis': "0 0 1"
    },
    'robot_v5(Mirror)_1+Motor_mount_v5(Mirror)_2+mount(Mirror)_1Revolute_2': {
        'axis': '0 1 0'
    },
    'left-leg_v10_1+Motor_mount_v7_1+mount_1Revolute_1': {
        'axis': "0 0 1"
    },
    'left-leg_v10_1+Motor_mount_v7_2+mount_1Revolute_3': {
        'axis': '0 1 0'
    },
    'body_1_mesh': {
        'density': '150'
    }
}

def _get_joints(component):
    joints = []
    for joint in component.joints:
        joints.append(joint)
    for joint in component.asBuiltJoints:
        joints.append(joint)

    for occurence in component.occurrences:
        for joint in _get_joints(occurence.component):
            joints.append(joint)
        
    return joints
        

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct
        if not design:
            ui.messageBox('No active design', 'No Design')
            return

        # --- 1. Get user input for root component name ---
        # (root_comp_name, cancelled) = ui.inputBox('Enter the name of the root component (e.g., torso)', 'Root Component Name')
        # if cancelled or not root_comp_name:
        #     return

        # --- 2. Find the root component occurrence ---
        root_comp_name = 'body'
        root_comp = design.rootComponent
        base_link_occurrence = None
        for occ in root_comp.occurrences:
            # Using component.name to match, as occurrence.name can be unique
            if occ.component.name == root_comp_name:
                base_link_occurrence = occ
                break
        
        if not base_link_occurrence:
            ui.messageBox(f'Component  not found as a direct occurrence in the root of the design.')
            return

        # --- 3. User Input for Export Directory ---
        # folder_dialog = ui.createFolderDialog()
        # folder_dialog.title = "Select Export Folder for MuJoCo Assets"
        # dialog_result = folder_dialog.showDialog()

        # if dialog_result == adsk.core.DialogResults.DialogOK:
        #     export_folder = folder_dialog.folder
        # else:
        #     return

        export_folder = '/Users/shpurcell/Documents/code/mujoco_test_export'
        mesh_folder = os.path.join(export_folder, 'meshes')
        if not os.path.exists(mesh_folder):
            os.makedirs(mesh_folder)

        # --- 4. Setup XML Structure ---
        mujoco_root = Element('mujoco', model=design.rootComponent.name.replace(" ", "_"))
        default_element = SubElement(mujoco_root, 'default')
        SubElement(default_element, 'mesh', scale="0.001 0.001 0.001")
        SubElement(default_element, 'geom', rgba="0.5 0.7 1.0 1")
        SubElement(default_element, 'joint', damping="1")
        SubElement(default_element, 'position', kp="14")

        asset_element = SubElement(mujoco_root, 'asset')
        actuator_element = SubElement(mujoco_root, 'actuator')
        worldbody_element = SubElement(mujoco_root, 'worldbody')
        
        # --- 5. Create a "world" body and base link ---
        SubElement(worldbody_element, 'geom', type='plane', size='1 1 0.1', rgba="0.9 0.9 0.9 1")
        SubElement(worldbody_element, 'light', mode="targetbodycom", target="body",diffuse=".8 .8 .8", specular="0.3 0.3 0.3", pos="0 -6 4", cutoff="30" )

        # Create the body for the base link
        base_link_body_element = SubElement(worldbody_element, 'body', name='body', pos='0 0 .17355')

        SubElement(base_link_body_element, 'freejoint', name="root")
        
        # Export the mesh for the base link
        export_mesh(base_link_occurrence, asset_element, base_link_body_element, mesh_folder, root_comp)

        # --- 6. Recursively build the robot tree from joints ---
        seen_links = set()

        every_joint = []
        for joint in root_comp.allJoints:
            every_joint.append(joint)
        for joint in root_comp.allAsBuiltJoints:
            every_joint.append(joint)

        build_robot_tree(base_link_occurrence, base_link_body_element, asset_element, mesh_folder, ui, root_comp, every_joint, seen_links, actuator_element)


        # --- 7. Write the XML file ---
        xml_string = tostring(mujoco_root, 'utf-8')
        reparsed = minidom.parseString(xml_string)
        pretty_xml = reparsed.toprettyxml(indent="  ")

        xml_file_path = os.path.join(export_folder, design.rootComponent.name.replace(" ", "_") + '.xml')
        with open(xml_file_path, "w") as f:
            f.write(pretty_xml)

        ui.messageBox(f'Successfully exported to {xml_file_path}')

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def build_robot_tree(parent_occurrence, parent_xml_element, asset_element, mesh_folder, ui, root_comp, all_joints, seen_links, actuator_element):
    """
    Recursively finds child components connected by joints and builds the XML structure.
    """
    for joint in all_joints:
        if not joint.occurrenceTwo:
            continue

        if (joint.occurrenceOne == parent_occurrence or joint.occurrenceTwo == parent_occurrence) and joint.entityToken not in seen_links:
            child_occurrence = joint.occurrenceTwo if joint.occurrenceTwo != parent_occurrence else joint.occurrenceOne

            print(f"found child!! {child_occurrence.fullPathName}")
            seen_links.add(joint.entityToken)

            
            # --- Create Child Body in XML ---
            child_body_name = child_occurrence.fullPathName.replace(':', '_').replace(' ', '_')
            child_body_element = SubElement(parent_xml_element, 'body', name=child_body_name)

            if isinstance(joint, adsk.fusion.Joint): 
                print(joint.name)
                # --- Get Joint Information for Child Body Position ---
                joint_origin = joint.geometryOrOriginOne.origin
                
                # Convert Fusion's mm to MuJoCo's meters for the body position
                pos_x = joint_origin.x / 100.0
                pos_y = joint_origin.y / 100.0
                pos_z = joint_origin.z / 100.0
                pos_str = f'{pos_x:.4f} {pos_y:.4f} {pos_z:.4f}'
                joint_name = child_body_name + joint.name.replace(':', '_').replace(' ', '_')
                # --- Create Joint inside Child Body ---
                # The joint is at the origin of the child body's frame
                if joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType:
                    SubElement(child_body_element, 'joint', 
                            name=joint_name, 
                            type=get_mujoco_joint_type(joint.jointMotion.jointType),
                            pos=pos_str,
                            attrib=_get_overrides({
                                    'axis': '1 0 0',
                                },joint_name)) # Default axis, you may need to derive this
                    SubElement(actuator_element, "position", name=joint_name, joint=joint_name)

            # --- Export Mesh for the Child Component and add geom ---
            export_mesh(child_occurrence, asset_element, child_body_element, mesh_folder, root_comp)

            build_robot_tree(child_occurrence, child_body_element, asset_element, mesh_folder, ui, root_comp, all_joints, seen_links, actuator_element)

def get_all_joints(component: adsk.fusion.Component):
    """Return a combined list of joints and as-built joints for a component."""
    all_joints = []
    
    # Standard joints
    for j in component.joints:
        all_joints.append(j)
    
    # As-built joints
    for aj in component.asBuiltJoints:
        all_joints.append(aj)
    
    return all_joints

def _get_overrides(attributes, key):
    overides = _overrides[key] if key in _overrides else {}
    return attributes | overides

def export_mesh(occurance, asset_element, body_element, mesh_folder, root_comp):
    """
    Exports a component's mesh as STL if it hasn't been exported yet.
    Adds the corresponding asset and geom tags to the XML.
    """
    occurance.isIsolated = True
    comp = occurance.component
    comp_name = occurance.fullPathName.replace(':', '_').replace(' ', '_')
    mesh_asset_name = f'{comp_name}_mesh'
    
    # Only export the mesh if we haven't done it for this component yet
    # if comp.name not in exported_meshes:
    mesh_filename = f"{comp_name}.stl"
    mesh_filepath = os.path.join(mesh_folder, mesh_filename)
    
    export_manager = adsk.fusion.ExportManager.cast(comp.parentDesign.exportManager)
    stl_options = export_manager.createSTLExportOptions(root_comp, mesh_filepath)
    stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    export_manager.execute(stl_options)

    # Add the mesh to the asset section
    SubElement(asset_element, 'mesh', file=f'meshes/{mesh_filename}', name=mesh_asset_name)



    SubElement(body_element, 'geom', 
               type='mesh', 
               mesh=mesh_asset_name,
               attrib=_get_overrides({},mesh_asset_name))
    occurance.isIsolated = False


def get_mujoco_joint_type(fusion_joint_type):
    """
    Maps Fusion 360 joint types to MuJoCo joint types.
    """
    if fusion_joint_type == adsk.fusion.JointTypes.RevoluteJointType:
        return 'hinge'
    elif fusion_joint_type == adsk.fusion.JointTypes.SliderJointType:
        return 'slide'
    elif fusion_joint_type == adsk.fusion.JointTypes.BallJointType:
        return 'ball'
    else:
        # Default to a hinge joint if type is not supported for simplicity
        return 'hinge'

