import bpy
import os

# Directory to save the individual .dae files
export_dir = bpy.path.abspath("//ExportedObjects_DAE/")

# Ensure the directory exists
if not os.path.exists(export_dir):
    os.makedirs(export_dir)

# Get all objects in the current scene
objects = bpy.context.scene.objects

for obj in objects:
    # Only export visible, selectable objects
    if obj.type != 'MESH':
        continue

    # Deselect all objects
    bpy.ops.object.select_all(action='DESELECT')

    # Select the current object
    obj.select_set(True)

    # Set the object as active
    bpy.context.view_layer.objects.active = obj

    # Create a new filename for the object
    filename = os.path.join(export_dir, f"{obj.name}.dae")
    
    # Export the selected object as a Collada file
    bpy.ops.wm.collada_export(filepath=filename, selected=True)

    print(f"Exported {obj.name} to {filename}")

print(f"Export complete! Files saved in {export_dir}")
