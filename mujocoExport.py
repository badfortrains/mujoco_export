# FUSION 360 SCRIPT
# DESCRIPTION: Exports a Fusion 360 design to a MuJoCo-compatible XML file.
# This script exports each component as an STL mesh and creates an XML file
# with joints positioned according to the Fusion 360 model.

import adsk.core, adsk.fusion, adsk.cam, traceback
import math
import os
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

SERVO_COMPONENT_NAME = 'servo v10'
SERVO_MASS_KG = 0.0134
SOLID_PLA_DENSITY_KG_M3 = 1240
PLA_INFILL_FRACTION = 0.15
PLA_15_PERCENT_DENSITY_KG_M3 = (
    SOLID_PLA_DENSITY_KG_M3 * PLA_INFILL_FRACTION)

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


def _joint_occurrences(joint):
    """Return the two occurrence proxies referenced by a joint."""
    return joint.occurrenceOne, joint.occurrenceTwo


def _find_joint_occurrence(all_joints, full_path_name):
    """Find an occurrence proxy by its path in the root assembly context."""
    for joint in all_joints:
        for occurrence in _joint_occurrences(joint):
            if occurrence and occurrence.fullPathName == full_path_name:
                return occurrence
    return None


def _joint_occurrence_paths(all_joints):
    """Return the occurrence paths referenced by the supplied joints."""
    return sorted({
        occurrence.fullPathName
        for joint in all_joints
        for occurrence in _joint_occurrences(joint)
        if occurrence
    })


def _format_axis_component(value):
    """Format an axis component compactly without negative zero."""
    if abs(value) < 1e-9:
        value = 0.0
    return f'{value:.6g}'


def _get_mujoco_joint_axis(joint_motion):
    """Return a normalized MuJoCo axis for supported Fusion joint motions."""
    if joint_motion.jointType != adsk.fusion.JointTypes.RevoluteJointType:
        return '1 0 0'

    axis = joint_motion.rotationAxisVector
    if not axis:
        return '1 0 0'

    length = math.sqrt(axis.x ** 2 + axis.y ** 2 + axis.z ** 2)
    if length < 1e-9:
        return '1 0 0'

    return ' '.join(
        _format_axis_component(component / length)
        for component in (axis.x, axis.y, axis.z)
    )


def _format_mujoco_vector(values):
    """Format a sequence of MuJoCo coordinates compactly."""
    return ' '.join(_format_axis_component(value) for value in values)


def _get_foot_box(occurrence):
    """Return the contact-box center, half-size, and sole center in meters."""
    # Fusion uses centimeters internally. The occurrence bounds are in the root
    # assembly context, matching the root-context STL export used below.
    try:
        bounding_box = occurrence.preciseBoundingBox
    except AttributeError:
        # preciseBoundingBox was added in January 2024. Keep the exporter usable
        # with older Fusion installations, where boundingBox is less exact.
        bounding_box = occurrence.boundingBox

    if not bounding_box:
        raise ValueError(
            f'Could not calculate a bounding box for {occurrence.fullPathName}')

    minimum = bounding_box.minPoint
    maximum = bounding_box.maxPoint
    center = tuple(
        (min_value + max_value) / 200.0
        for min_value, max_value in zip(
            (minimum.x, minimum.y, minimum.z),
            (maximum.x, maximum.y, maximum.z)))
    half_size = tuple(
        (max_value - min_value) / 200.0
        for min_value, max_value in zip(
            (minimum.x, minimum.y, minimum.z),
            (maximum.x, maximum.y, maximum.z)))
    sole_center = (center[0], center[1], minimum.z / 100.0)

    if any(size <= 0 for size in half_size):
        raise ValueError(
            f'Foot occurrence has an empty bounding box: {occurrence.fullPathName}')

    return center, half_size, sole_center


def add_foot_contact_geometry(occurrence, body_element, side):
    """Add a collidable box and a sole-center site for one foot occurrence."""
    center, half_size, sole_center = _get_foot_box(occurrence)

    # density=0 prevents the collision proxy from adding mass on top of the
    # visual mesh. contype/conaffinity override the collision-disabled default.
    SubElement(
        body_element,
        'geom',
        name=f'{side}_foot_collision',
        type='box',
        pos=_format_mujoco_vector(center),
        size=_format_mujoco_vector(half_size),
        contype='1',
        conaffinity='1',
        rgba='1 0 0 1',
        density='0')
    SubElement(
        body_element,
        'site',
        name=f'{side}_foot_center',
        pos=_format_mujoco_vector(sole_center),
        size='0.003',
        type='sphere',
        rgba='1 0 0 1')
        

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct
        if not design:
            ui.messageBox('No active design', 'No Design')
            return

        # --- 1. Configure the occurrence paths that make up the base link ---
        base_link_full_paths = [
            'body:1+motorMountU v7:3',
            'body:1+motorMountU v7(Mirror):1'
        ]

        # These are the two sole occurrences. Their tight B-Rep bounds are used
        # to generate the foot contact boxes and sole-center sites.
        foot_full_paths = {
            'foot:1+Component54:1': 'left',
            'foot(Mirror) (1):1+Component54(Mirror) (1):1': 'right'
        }

        # --- 2. Find the base occurrences through the root-context joints ---
        root_comp = design.rootComponent

        every_joint = []
        for joint in root_comp.allJoints:
            every_joint.append(joint)
        for joint in root_comp.allAsBuiltJoints:
            every_joint.append(joint)

        base_link_occurrences = []
        missing_base_link_paths = []
        for full_path in dict.fromkeys(base_link_full_paths):
            occurrence = _find_joint_occurrence(every_joint, full_path)
            if occurrence:
                base_link_occurrences.append(occurrence)
            else:
                missing_base_link_paths.append(full_path)

        if not base_link_full_paths:
            ui.messageBox('At least one base-link occurrence path is required.')
            return

        if missing_base_link_paths:
            missing_paths = '\n'.join(missing_base_link_paths)
            referenced_paths = '\n'.join(_joint_occurrence_paths(every_joint))
            ui.messageBox(
                f'No joint references these base-link occurrences:\n{missing_paths}'
                f'\n\nOccurrence paths referenced by joints:\n{referenced_paths}')
            return

        missing_foot_paths = [
            full_path for full_path in foot_full_paths
            if not _find_joint_occurrence(every_joint, full_path)
        ]
        if missing_foot_paths:
            missing_paths = '\n'.join(missing_foot_paths)
            referenced_paths = '\n'.join(_joint_occurrence_paths(every_joint))
            ui.messageBox(
                f'No joint references these foot occurrences:\n{missing_paths}'
                f'\n\nOccurrence paths referenced by joints:\n{referenced_paths}')
            return

        # --- 3. User Input for Export Directory ---
        # folder_dialog = ui.createFolderDialog()
        # folder_dialog.title = "Select Export Folder for MuJoCo Assets"
        # dialog_result = folder_dialog.showDialog()

        # if dialog_result == adsk.core.DialogResults.DialogOK:
        #     export_folder = folder_dialog.folder
        # else:
        #     return

        export_folder = '/Users/suzanna/Documents/code/rick_v2'
        mesh_folder = os.path.join(export_folder, 'meshes')
        if not os.path.exists(mesh_folder):
            os.makedirs(mesh_folder)

        # --- 4. Setup XML Structure ---
        mujoco_root = Element('mujoco', model=design.rootComponent.name.replace(" ", "_"))
        SubElement(mujoco_root, 'compiler', angle='radian')
        default_element = SubElement(mujoco_root, 'default')
        SubElement(default_element, 'mesh', scale="0.001 0.001 0.001")
        SubElement(
            default_element,
            'geom',
            rgba='0.5 0.7 1.0 1',
            contype='0',
            conaffinity='0')
        SubElement(
            default_element,
            'joint',
            damping='0.002',
            range='-1.5 1.5',
            armature='0.0005')
        SubElement(
            default_element,
            'position',
            kp='5',
            kv='0.02',
            forcelimited='true',
            forcerange='-0.2 0.2',
            ctrllimited='true',
            ctrlrange='-1.57 1.57')

        asset_element = SubElement(mujoco_root, 'asset')
        actuator_element = SubElement(mujoco_root, 'actuator')
        worldbody_element = SubElement(mujoco_root, 'worldbody')
        
        # --- 5. Create a "world" body and base link ---
        SubElement(
            worldbody_element,
            'geom',
            name='floor',
            size='0 0 .05',
            type='plane',
            condim='3',
            rgba='0.9 0.9 0.9 1',
            contype='1',
            conaffinity='1')
        SubElement(worldbody_element, 'light', mode="targetbodycom", target="body",diffuse=".8 .8 .8", specular="0.3 0.3 0.3", pos="0 -6 4", cutoff="30" )

        # Create the body for the base link
        base_link_body_element = SubElement(worldbody_element, 'body', name='body')

        SubElement(base_link_body_element, 'freejoint', name="root")
        
        # Export each occurrence that makes up the base link into the same XML body.
        for base_link_occurrence in base_link_occurrences:
            export_mesh(base_link_occurrence, asset_element, base_link_body_element, mesh_folder, root_comp)

        # --- 6. Recursively build the robot tree from joints ---
        seen_links = set()

        for base_link_occurrence in base_link_occurrences:
            build_robot_tree(base_link_occurrence, base_link_body_element, asset_element, mesh_folder, ui, root_comp, every_joint, seen_links, actuator_element, foot_full_paths)

        SubElement(
            worldbody_element,
            'camera',
            name='track',
            mode='trackcom',
            pos='0 -2.1 0.5',
            xyaxes='1 0 0 0 0.2 1',
            target='body')


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

def build_robot_tree(parent_occurrence, parent_xml_element, asset_element, mesh_folder, ui, root_comp, all_joints, seen_links, actuator_element, foot_full_paths):
    """
    Recursively finds child components connected by joints and builds the XML structure.
    """
    parent_path = parent_occurrence.fullPathName

    for joint in all_joints:
        occurrence_one, occurrence_two = _joint_occurrences(joint)
        if not occurrence_one or not occurrence_two:
            continue

        occurrence_one_path = occurrence_one.fullPathName
        occurrence_two_path = occurrence_two.fullPathName
        print(f"occurenceOne : {occurrence_one_path} occurance two: {occurrence_two_path}")

    for joint in all_joints:
        occurrence_one, occurrence_two = _joint_occurrences(joint)
        if not occurrence_one or not occurrence_two:
            continue

        occurrence_one_path = occurrence_one.fullPathName
        occurrence_two_path = occurrence_two.fullPathName
        if (parent_path in (occurrence_one_path, occurrence_two_path)
                and joint.entityToken not in seen_links):
            child_occurrence = (occurrence_two
                                if occurrence_one_path == parent_path
                                else occurrence_one)

            print(f"found child!! {child_occurrence.fullPathName}")
            seen_links.add(joint.entityToken)

            
            # --- Create Child Body in XML ---
            child_body_name = child_occurrence.fullPathName.replace(':', '_').replace(' ', '_')
            child_body_element = SubElement(parent_xml_element, 'body', name=child_body_name)

            if isinstance(joint, adsk.fusion.Joint): 
                print(joint.name)
                print(parent_path)
                # --- Get Joint Information for Child Body Position ---

                joint_name = child_body_name + joint.name.replace(':', '_').replace(' ', '_')
                # --- Create Joint inside Child Body ---
                # The joint is at the origin of the child body's frame
                if joint.jointMotion.jointType != adsk.fusion.JointTypes.RigidJointType:
                    joint_origin = joint.geometryOrOriginOne.origin
                    # Convert Fusion's mm to MuJoCo's meters for the body position
                    pos_x = joint_origin.x / 100.0
                    pos_y = joint_origin.y / 100.0
                    pos_z = joint_origin.z / 100.0
                    pos_str = f'{pos_x:.4f} {pos_y:.4f} {pos_z:.4f}'
                    joint_axis = _get_mujoco_joint_axis(joint.jointMotion)
                    SubElement(child_body_element, 'joint', 
                            name=joint_name, 
                            type=get_mujoco_joint_type(joint.jointMotion.jointType),
                            pos=pos_str,
                            attrib=_get_overrides({
                                    'axis': joint_axis,
                                },joint_name))
                    SubElement(actuator_element, "position", name=joint_name, joint=joint_name)

            # --- Export Mesh for the Child Component and add geom ---
            export_mesh(child_occurrence, asset_element, child_body_element, mesh_folder, root_comp)

            foot_side = foot_full_paths.get(child_occurrence.fullPathName)
            if foot_side:
                add_foot_contact_geometry(
                    child_occurrence, child_body_element, foot_side)

            build_robot_tree(child_occurrence, child_body_element, asset_element, mesh_folder, ui, root_comp, all_joints, seen_links, actuator_element, foot_full_paths)

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


def _normalize_fusion_component_name(name):
    """Remove Fusion instance, mirror, and duplicate-name suffixes."""
    component_name = name.strip().casefold()

    while True:
        previous_name = component_name

        name_without_instance, separator, instance_number = (
            component_name.rpartition(':'))
        if separator and instance_number.isdigit():
            component_name = name_without_instance.rstrip()

        if component_name.endswith(')'):
            suffix_start = component_name.rfind('(')
            duplicate_number = component_name[suffix_start + 1:-1]
            if suffix_start >= 0 and duplicate_number.isdigit():
                component_name = component_name[:suffix_start].rstrip()

        print(f"stripped name {component_name}")
        mirror_suffix = '(mirror)'
        if component_name.endswith(mirror_suffix):
            component_name = component_name[:-len(mirror_suffix)].rstrip()

        if component_name == previous_name:
            return component_name


def _is_servo_occurrence(occurrence):
    """Return whether an occurrence is an original or renamed mirror servo."""
    component_names = [occurrence.component.name]
    if occurrence.fullPathName:
        component_names.append(occurrence.fullPathName.rsplit('+', 1)[-1])

    normalized_names = [
        _normalize_fusion_component_name(name) for name in component_names]
    print(normalized_names)
    return SERVO_COMPONENT_NAME.casefold() in normalized_names


def _get_mesh_mass_attributes(occurrence):
    """Return MuJoCo mass properties for an exported component occurrence."""
    if _is_servo_occurrence(occurrence):
        # MuJoCo mass values use kilograms.
        return {'mass': f'{SERVO_MASS_KG:g}'}

    # MuJoCo density values use kg/m^3. This treats the infill percentage as
    # the printed part's solid-volume fraction.
    return {'density': f'{PLA_15_PERCENT_DENSITY_KG_M3:g}'}


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
               attrib=_get_overrides(
                   _get_mesh_mass_attributes(occurance), mesh_asset_name))
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
