import trimesh
import os

# Define your meshes
meshes = {
    'tomato': '/home/guru-vignesh/Downloads/tomato/scene.gltf',
    #'onion': '/home/guru-vignesh/Downloads/onion/scene.gltf',
    #'cabbage': '/home/guru-vignesh/Documents/cabbage/scene.gltf',
    #'table': '/home/guru-vignesh/Downloads/dining_table/scene.gltf',
   # 'plate': '/home/guru-vignesh/Documents/plate/scene.gltf',
   #'bell': '/home/guru-vignesh/Documents/red_bell_pepper/scene.gltf',
}

output_dir = '/home/guru-vignesh/iiit/meshes/'
os.makedirs(output_dir, exist_ok=True)

for name, gltf_path in meshes.items():
    if os.path.exists(gltf_path):
        print(f"Converting {name}...")
        mesh = trimesh.load(gltf_path)
        
        # Handle scenes with multiple meshes
        if isinstance(mesh, trimesh.Scene):
            mesh = trimesh.util.concatenate(
                [m for m in mesh.geometry.values()]
            )
        
        # Export as OBJ
        output_path = os.path.join(output_dir, f'{name}.obj')
        mesh.export(output_path)
        print(f"  ✓ Saved to {output_path}")
    else:
        print(f"  ✗ {gltf_path} not found")

print("\nDone! Update your XML to use these paths.")

