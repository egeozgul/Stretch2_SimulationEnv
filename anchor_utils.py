"""Utility functions for reading anchor positions from XML file."""

import os
import re
import numpy as np


def load_anchors_from_xml(xml_path=None):
    """
    Load anchor positions from MuJoCo XML file.
    Uses regex parsing to avoid XML parsing issues with malformed attributes.
    
    Args:
        xml_path: Path to XML file. If None, looks for table_world.xml in script directory.
    
    Returns:
        dict: Dictionary mapping anchor letters to positions {'A': [x, y, z], ...}
    """
    if xml_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = os.path.join(script_dir, 'table_world.xml')
    
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"XML file not found: {xml_path}")
    
    anchors = {}
    
    # Read file and use regex to find anchor sites
    with open(xml_path, 'r') as f:
        content = f.read()
    
    # Pattern to match: <site name="anchor_X" pos="x y z" ... />
    # This handles malformed XML better than ET.parse
    pattern = r'<site\s+name="anchor_([A-Za-z])"[^>]*pos="([^"]+)"'
    
    for match in re.finditer(pattern, content):
        anchor_letter = match.group(1).upper()
        pos_str = match.group(2)
        
        # Parse position string (format: "x y z" or "x y")
        try:
            pos_values = [float(x) for x in pos_str.split()]
            if len(pos_values) >= 2:
                # Use x, y, z (pad with 0.0 if z not provided)
                pos = pos_values[:3] if len(pos_values) >= 3 else pos_values + [0.0]
                anchors[anchor_letter] = pos
        except ValueError:
            continue  # Skip invalid positions
    
    return anchors


def get_anchor_list(xml_path=None):
    """
    Get list of available anchor letters.
    
    Args:
        xml_path: Path to XML file. If None, looks for table_world.xml in script directory.
    
    Returns:
        list: Sorted list of anchor letters ['A', 'B', 'C', ...]
    """
    anchors = load_anchors_from_xml(xml_path)
    return sorted(anchors.keys())

