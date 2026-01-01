"""
Utility module for resolving paths relative to the repository root.
This ensures all scripts work regardless of where they are executed from.
"""
import os


def get_repo_root():
    """Get the absolute path to the repository root directory."""
    # This file is in the repo root, so its directory is the root
    return os.path.dirname(os.path.abspath(__file__))


def get_path(*path_parts):
    """
    Get an absolute path relative to the repository root.
    
    Args:
        *path_parts: Path components relative to repo root
        
    Returns:
        str: Absolute path
        
    Examples:
        get_path('meshes', 'table.obj') -> '/path/to/repo/meshes/table.obj'
        get_path('table_world.xml') -> '/path/to/repo/table_world.xml'
    """
    return os.path.join(get_repo_root(), *path_parts)


def get_xml_path(filename='table_world.xml'):
    """Get the path to an XML file in the repo root."""
    return get_path(filename)


def get_mesh_path(filename):
    """Get the path to a mesh file in the meshes directory."""
    return get_path('meshes', filename)


def get_texture_path(filename):
    """Get the path to a texture file in the textures directory."""
    return get_path('textures', filename)


def get_asset_path(filename):
    """Get the path to an asset file in the assets directory."""
    return get_path('assets', filename)

