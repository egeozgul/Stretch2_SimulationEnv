"""Utility functions for reading anchor positions from XML file."""

import os
import re
import math
import numpy as np


def load_anchors_from_xml(xml_path=None):
    """
    Load anchor positions and directions from MuJoCo XML file.
    
    Returns:
        dict: {'A': {'pos': [x, y, z], 'direction': angle_rad}, ...}
    """
    if xml_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = os.path.join(script_dir, 'table_world.xml')
    
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"XML file not found: {xml_path}")
    
    with open(xml_path, 'r') as f:
        content = f.read()
    
    anchors = {}
    pattern = r'<site\s+name="anchor_([A-Za-z])"[^>]*pos="([^"]+)"'
    
    for match in re.finditer(pattern, content):
        anchor_letter = match.group(1).upper()
        pos_str = match.group(2)
        
        # Look for direction in comment
        comment_pattern = rf'<!--\s*anchor_{anchor_letter}\s+direction="([^"]+)"\s*-->'
        direction_match = re.search(comment_pattern, content, re.IGNORECASE)
        direction_str = direction_match.group(1) if direction_match else None
        
        try:
            pos_values = [float(x) for x in pos_str.split()]
            if len(pos_values) >= 2:
                pos = pos_values[:3] if len(pos_values) >= 3 else pos_values + [0.0]
                anchor_data = {'pos': pos}
                
                if direction_str:
                    try:
                        anchor_data['direction'] = math.radians(float(direction_str))
                    except ValueError:
                        pass
                
                anchors[anchor_letter] = anchor_data
        except ValueError:
            continue
    
    return anchors


def get_anchor_list(xml_path=None):
    """Get sorted list of available anchor letters."""
    anchors = load_anchors_from_xml(xml_path)
    return sorted(anchors.keys())
