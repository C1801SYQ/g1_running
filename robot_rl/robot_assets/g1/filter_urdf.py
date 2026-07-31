"""Properly filter the hand URDF to remove finger joints and their child links."""
import re, sys

with open('/home/ubuntu/robot_rl/transfer/obelisk/g1_model/urdf/g1_hand_renamed.urdf') as f:
    content = f.read()

# Finger-related patterns to remove
finger_patterns = [
    r'<joint\b[^>]*name="[^"]*hand_thumb[^"]*"[^>]*>.*?</joint>',
    r'<joint\b[^>]*name="[^"]*hand_index[^"]*"[^>]*>.*?</joint>',
    r'<joint\b[^>]*name="[^"]*hand_middle[^"]*"[^>]*>.*?</joint>',
    r'<link\b[^>]*name="[^"]*hand_thumb[^"]*"[^>]*>.*?</link>',
    r'<link\b[^>]*name="[^"]*hand_index[^"]*"[^>]*>.*?</link>',
    r'<link\b[^>]*name="[^"]*hand_middle[^"]*"[^>]*>.*?</link>',
    r'<link\b[^>]*name="[^"]*hand_palm[^"]*"[^>]*>.*?</link>',
    r'<link\b[^>]*name="[^"]*rubber_hand[^"]*"[^>]*>.*?</link>',
]

for pattern in finger_patterns:
    content = re.sub(pattern, '', content, flags=re.DOTALL)

# Clean up multiple blank lines
content = re.sub(r'\n\s*\n\s*\n', '\n\n', content)

out = '/home/ubuntu/robot_rl/robot_assets/g1/g1_29dof_clean.urdf'
with open(out, 'w') as f:
    f.write(content)

print(f'Saved: {out}')
# Count remaining revolute joints
joints = re.findall(r'joint.*?name="(\w+)".*?type="revolute"', content)
print(f'Revolute joints ({len(joints)}):')
for j in joints:
    print(f'  {j}')
