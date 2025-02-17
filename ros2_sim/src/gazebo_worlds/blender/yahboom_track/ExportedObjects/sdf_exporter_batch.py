import bpy
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Function to extract the base name (remove suffixes like `.001`)
def get_base_name(name):
    return name.split('.')[0]

# Function to export model files
def export_sdf(prefix_path, object_name):
    # Extract the base name for folder creation
    base_name = get_base_name(object_name)

    # Define the folder structure and filenames
    base_folder = os.path.join(prefix_path, "YahboomTrack")
    object_folder = os.path.join(base_folder, base_name)
    meshes_folder = os.path.join(object_folder, "meshes")
    os.makedirs(meshes_folder, exist_ok=True)

    dae_filename = f'{base_name}.dae'
    sdf_filename = 'model.sdf'
    model_config_filename = 'model.config'

    # Export the .dae file for the current object
    bpy.ops.wm.collada_export(
        filepath=os.path.join(meshes_folder, dae_filename), 
        check_existing=False
    )

    #############################################
    #### Export SDF XML for the object ####
    #############################################
    sdf = ET.Element('sdf', attrib={'version': '1.8'})

    # Model and link structure
    model = ET.SubElement(sdf, "model", attrib={"name": base_name})
    static = ET.SubElement(model, "static")
    static.text = "true"
    link = ET.SubElement(model, "link", attrib={"name": base_name})

    # Set the scale factor
    scale_factor = 2.0

    # Visual element
    visual = ET.SubElement(link, "visual", attrib={"name": "visual"})
    geometry_visual = ET.SubElement(visual, "geometry")
    mesh_visual = ET.SubElement(geometry_visual, "mesh")
    uri_visual = ET.SubElement(mesh_visual, "uri")
    uri_visual.text = f"model://YahboomTrack/{base_name}/meshes/{dae_filename}"
    scale_visual = ET.SubElement(mesh_visual, "scale")
    scale_visual.text = f"{scale_factor} {scale_factor} {scale_factor}"

    # Collision element
    collision = ET.SubElement(link, "collision", attrib={"name": "visual"})
    geometry_collision = ET.SubElement(collision, "geometry")
    mesh_collision = ET.SubElement(geometry_collision, "mesh")
    uri_collision = ET.SubElement(mesh_collision, "uri")
    uri_collision.text = f"model://YahboomTrack/{base_name}/meshes/{dae_filename}"
    scale_collision = ET.SubElement(mesh_collision, "scale")
    scale_collision.text = f"{scale_factor} {scale_factor} {scale_factor}"

    # Write the SDF file
    sdf_file_path = os.path.join(object_folder, sdf_filename)
    with open(sdf_file_path, "w") as sdf_file:
        xml_string = ET.tostring(sdf, encoding='unicode')
        reparsed = minidom.parseString(xml_string)
        sdf_file.write(reparsed.toprettyxml(indent="  "))

    ##############################
    ### Generate model.config ####
    ##############################
    model_config = ET.Element('model')
    name = ET.SubElement(model_config, 'name')
    name.text = base_name
    sdf_tag = ET.SubElement(model_config, "sdf", attrib={"version": "1.8"})
    sdf_tag.text = sdf_filename

    # Write the model.config file
    model_config_file_path = os.path.join(object_folder, model_config_filename)
    with open(model_config_file_path, "w") as config_file:
        xml_string = ET.tostring(model_config, encoding='unicode')
        reparsed = minidom.parseString(xml_string)
        config_file.write(reparsed.toprettyxml(indent="  "))

    print(f"Export complete! Files for {base_name} saved in {object_folder}")

# Automatically create a folder for each object
def create_output_folder():
    blend_file_path = bpy.data.filepath
    if not blend_file_path:
        raise RuntimeError("Save the .blend file before running this script.")

    # Derive the base output folder path
    blend_dir = os.path.dirname(blend_file_path)
    return blend_dir

# Main function
def main():
    output_folder = create_output_folder()

    # Export each object as a separate folder inside "YahboomTrack"
    objects = bpy.context.selectable_objects
    mesh_objects = [o for o in objects if o.type == 'MESH']

    for obj in mesh_objects:
        # Select and isolate the current object for export
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

        export_sdf(output_folder, obj.name)

if __name__ == "__main__":
    main()
