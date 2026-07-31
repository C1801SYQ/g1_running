"""Filter hand URDF: keep only 29 joints, remove finger joints and their child links."""
import xml.etree.ElementTree as ET
import copy

FINGER_JOINTS = {
    "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
    "left_hand_middle_0_joint", "left_hand_middle_1_joint",
    "left_hand_index_0_joint", "left_hand_index_1_joint",
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_middle_0_joint", "right_hand_middle_1_joint",
    "right_hand_index_0_joint", "right_hand_index_1_joint",
}
FINGER_LINKS = {
    "left_hand_thumb_0_link", "left_hand_thumb_1_link", "left_hand_thumb_2_link",
    "left_hand_middle_0_link", "left_hand_middle_1_link",
    "left_hand_index_0_link", "left_hand_index_1_link",
    "right_hand_thumb_0_link", "right_hand_thumb_1_link", "right_hand_thumb_2_link",
    "right_hand_middle_0_link", "right_hand_middle_1_link",
    "right_hand_index_0_link", "right_hand_index_1_link",
    "left_hand_palm_link", "right_hand_palm_link",
    "left_rubber_hand", "right_rubber_hand",
}

tree = ET.parse('/home/ubuntu/robot_rl/transfer/obelisk/g1_model/urdf/g1_hand_renamed.urdf')
root = tree.getroot()

# Collect parent-child relationships
for joint in list(root.iter('joint')):
    name = joint.get('name')
    if name in FINGER_JOINTS:
        # Find child link of this joint and remove it too
        child_elem = joint.find('child')
        if child_elem is not None:
            child_link_name = child_elem.get('link')
            if child_link_name:
                FINGER_LINKS.add(child_link_name)
        # Remove the joint
        parent_map = {c: p for p in root.iter() for c in p}
        if joint in parent_map:
            parent_map[joint].remove(joint)
        print(f"Removed joint: {name}")

# Remove finger links (including any nested sub-links and joints recursively)
removed = set()
while True:
    new_removes = set()
    for link in list(root.iter('link')):
        name = link.get('name')
        if name in FINGER_LINKS and name not in removed:
            # Check if any joint references this link as parent
            for joint in list(root.iter('joint')):
                parent_elem = joint.find('parent')
                if parent_elem is not None and parent_elem.get('link') == name:
                    # Remove this joint too
                    FINGER_JOINTS.add(joint.get('name'))
                    parent_map = {c: p for p in root.iter() for c in p}
                    if joint in parent_map:
                        parent_map[joint].remove(joint)
                    print(f"  Also removed joint: {joint.get('name')} (parent was {name})")
            root.remove(link)
            removed.add(name)
            print(f"Removed link: {name}")
            new_removes.add(name)
    if not new_removes:
        break

output_path = '/home/ubuntu/robot_rl/robot_assets/g1/g1_29dof_clean.urdf'
tree.write(output_path, encoding='unicode')
print(f"\nSaved filtered URDF to: {output_path}")

# Count remaining joints
tree2 = ET.parse(output_path)
joints = [j.get('name') for j in tree2.iter('joint') if j.get('type') == 'revolute']
print(f"Remaining revolute joints ({len(joints)}): {joints}")
