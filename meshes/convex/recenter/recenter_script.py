import numpy as np
from stl import mesh

def compute_centroid(stl_mesh):
    """Compute the average of all vertices in the mesh."""
    # Get all unique vertices
    vertices = stl_mesh.vectors.reshape(-1, 3)
    centroid = np.mean(vertices, axis=0)
    return centroid

def translate_mesh(stl_mesh, translation_vector):
    """Apply translation vector to all vertices of the mesh."""
    # Apply translation to each triangle's vertices
    stl_mesh.vectors += translation_vector
    return stl_mesh

def main():
    # File paths
    file_a = 'tomato_convex_001.stl'  # A
    file_b = 'tomato_convex_000.stl'  # B
    file_c = 'tomato_convex_combined.stl'  # C
    
    # Load STL files
    print("Loading STL files...")
    mesh_a = mesh.Mesh.from_file(file_a)
    mesh_b = mesh.Mesh.from_file(file_b)
    mesh_c = mesh.Mesh.from_file(file_c)
    
    # Compute centroid of A
    print("Computing centroid of object A...")
    a_origin = compute_centroid(mesh_a)
    print(f"A_origin: {a_origin}")
    
    # Calculate translation vector (A_origin - A means we subtract A_origin from all vertices)
    # This centers A at the origin
    translation_vector = -a_origin
    print(f"Translation vector: {translation_vector}")
    
    # Apply translation to all three meshes
    print("Applying translation to objects A, B, and C...")
    mesh_a = translate_mesh(mesh_a, translation_vector)
    mesh_b = translate_mesh(mesh_b, translation_vector)
    mesh_c = translate_mesh(mesh_c, translation_vector)
    
    # Save the translated meshes
    print("Saving translated STL files...")
    mesh_a.save('tomato_convex_001_recentered.stl')
    mesh_b.save('tomato_convex_000_recentered.stl')
    mesh_c.save('tomato_convex_combined.stl')
    
    print("Done! Files saved as A_translated.stl, B_translated.stl, C_translated.stl")

if __name__ == "__main__":
    main()