import trimesh
import numpy as np

# Load mesh
mesh = trimesh.load('/home/guru-vignesh/iiit/meshes/table.obj')

print("BEFORE FIX:")
print(f"  Dimensions: {mesh.extents}")
print(f"  Bounds: {mesh.bounds}")

# Fix 1: Scale down (200 units → 2 meters)
scale_factor = 1.3
mesh.apply_scale(scale_factor)
print(f"\n✓ Scaled by {scale_factor}")

# Fix 2: Move bottom to Z=0
min_z = mesh.bounds[0][2]
mesh.apply_translation([0, 0, -min_z])
print(f"✓ Moved to Z=0 (translated by {-min_z:.3f})")

# Fix 3: Center X and Y at origin
centroid = mesh.centroid
mesh.apply_translation([-centroid[0], -centroid[1], 0])
print(f"✓ Centered at origin")

print("\nAFTER FIX:")
print(f"  Dimensions: {mesh.extents}")
print(f"  Bounds: {mesh.bounds}")
print(f"  Table height: {mesh.extents[2]:.3f}m")

# Save fixed mesh
mesh.export('/home/guru-vignesh/iiit/meshes/table_fixed.obj')
print(f"\n✓ Saved to table_fixed.obj")